#-----------------------------------------------------------------------------
#
# Macro: imstk_add_external_project
#
# Goal:  Wraps ExternalProject_Include_Dependencies and ExternalProject_add.
#
# Parsed arguments :
# - SOURCE_DIR : location of the external project sources. If not specified,
#   it defaults to "External/EXTERNAL_PROJECT_NAME/src"
# - RELATIVE_INCLUDE_PATH : path to the project include files relatively t
#   SOURCE_DIR (above). Used to populate the variable CMAKE_INCLUDE_PATH.
# - DEPENDENCIES : Name of dependencies needed by this project.
#
# Other arguments : any other arguments will be passed to ExternalProject_Add
# directly. Look at ExternalProject_Add documentation for more inputs info.
#
#-----------------------------------------------------------------------------

set(SKIP_STEP_COMMAND ${CMAKE_COMMAND} -E echo "Skip step")

macro(imstk_add_external_project extProj)

  #-----------------------------------------------------------------------------
  # Parse arguments
  #-----------------------------------------------------------------------------
  set(options VERBOSE)
  set(oneValueArgs RELATIVE_INCLUDE_PATH SOURCE_DIR BINARY_DIR)
  set(multiValueArgs DEPENDENCIES)
  include(CMakeParseArguments)
  cmake_parse_arguments(${extProj} "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  #-----------------------------------------------------------------------------
  # Verbose (display arguments)
  #-----------------------------------------------------------------------------
  if(${extProj}_VERBOSE)
    foreach(opt ${options} ${oneValueArgs} ${multiValueArgs})
      message(STATUS "${extProj}_${opt}: ${${extProj}_${opt}}")
    endforeach()
    message(STATUS "${extProj}_UNPARSED_ARGUMENTS: ${${extProj}_UNPARSED_ARGUMENTS}")
  endif()

  #-----------------------------------------------------------------------------
  # Sanity checks
  #-----------------------------------------------------------------------------
  if(DEFINED ${extProj}_DIR AND NOT EXISTS ${${extProj}_DIR})
    message(FATAL_ERROR
      "${extProj}_DIR variable is defined but corresponds to non-existing directory")
  endif()

  #-----------------------------------------------------------------------------
  # Solve dependencies
  #-----------------------------------------------------------------------------
  set(PROJECT_VAR_TEMP ${extProj})
  ExternalProject_Include_Dependencies( ${extProj}
    PROJECT_VAR PROJECT_VAR_TEMP
    EP_ARGS_VAR ${extProj}_EP_ARGS
    DEPENDS_VAR ${extProj}_DEPENDENCIES
    USE_SYSTEM_VAR USE_SYSTEM_${extProj}
    SUPERBUILD_VAR ${PROJECT_NAME}_SUPERBUILD
    )

  #-----------------------------------------------------------------------------
  # If needs to download and build
  #-----------------------------------------------------------------------------
  if(NOT DEFINED ${extProj}_DIR AND NOT ${USE_SYSTEM_${extProj}})

    #-----------------------------------------------------------------------------
    # Set project directory
    #-----------------------------------------------------------------------------
    set(${extProj}_PREFIX "${CMAKE_BINARY_DIR}/External/${extProj}")
    if("${${extProj}_SOURCE_DIR}" STREQUAL "")
      set(${extProj}_SOURCE_DIR "${${extProj}_PREFIX}/src")
    endif()
    if("${${extProj}_BINARY_DIR}" STREQUAL "")
      set(${extProj}_BINARY_DIR "${${extProj}_PREFIX}/build")
    endif()
    set(${extProj}_TMP_DIR "${${extProj}_PREFIX}/tmp")
    set(${extProj}_STAMP_DIR "${${extProj}_PREFIX}/stamp")

    #-----------------------------------------------------------------------------
    # Add project
    #-----------------------------------------------------------------------------

    set(ep_common_c_flags "${CMAKE_C_FLAGS_INIT} ${ADDITIONAL_C_FLAGS}")
    set(ep_common_cxx_flags "${CMAKE_CXX_FLAGS_INIT} ${ADDITIONAL_CXX_FLAGS}")

    ExternalProject_add( ${extProj}
      PREFIX ${${extProj}_PREFIX}
      SOURCE_DIR ${${extProj}_SOURCE_DIR} # from above or parsed argument
      BINARY_DIR ${${extProj}_BINARY_DIR} # from above or parsed argument
      TMP_DIR ${${extProj}_TMP_DIR}       # from above
      STAMP_DIR ${${extProj}_STAMP_DIR}   # from above
      ${${extProj}_EP_ARGS}               # from ExternalProject_Include_Dependencies
      ${${extProj}_UNPARSED_ARGUMENTS}    # from unparsed arguments of this macro
      CMAKE_CACHE_ARGS
        -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
        -DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
        -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
        -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
        -DCMAKE_CXX_STANDARD_REQUIRED:BOOL=${CMAKE_CXX_STANDARD_REQUIRED}
        -DCMAKE_CXX_EXTENSIONS:BOOL=${CMAKE_CXX_EXTENSIONS}
      DEPENDS ${${extProj}_DEPENDENCIES}  # from parsed argument
      )
      set(${extProj}_DIR ${${extProj}_BINARY_DIR})

  #-----------------------------------------------------------------------------
  # If project already built on system
  #-----------------------------------------------------------------------------
  else()

    #-----------------------------------------------------------------------------
    # Find package if USE_SYSTEM
    #-----------------------------------------------------------------------------
    if( ${USE_SYSTEM_${extProj}} )
      find_package( ${extProj} REQUIRED )
    endif()

    #-----------------------------------------------------------------------------
    # Add empty project (to solve dependencies)
    #-----------------------------------------------------------------------------
    ExternalProject_Add_Empty(${extProj} DEPENDS ${${extProj}_DEPENDENCIES})

  endif()

  #-----------------------------------------------------------------------------
  # Keep track of include path for superbuild
  #-----------------------------------------------------------------------------
  list(APPEND CMAKE_INCLUDE_PATH
     ${${extProj}_SOURCE_DIR}/${${extProj}_RELATIVE_INCLUDE_PATH}
     )

  #-----------------------------------------------------------------------------
  # Keep track of build path to add it in the innerbuild cache
  #-----------------------------------------------------------------------------
  mark_as_superbuild(${extProj}_DIR:PATH)
endmacro()
