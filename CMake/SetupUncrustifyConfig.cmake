#-----------------------------------------------------------------------------
# Define configuration files
#-----------------------------------------------------------------------------
set(Uncrustify_CONFIG "${CMAKE_SOURCE_DIR}/Utilities/Uncrustify/${PROJECT_NAME}Uncrustify.cfg")

#-----------------------------------------------------------------------------
# Setup git hook for Uncrustify executable path
#-----------------------------------------------------------------------------
find_package(Git)
if(GIT_FOUND)
  if(EXISTS "${Uncrustify_EXECUTABLE}" AND EXISTS "${Uncrustify_CONFIG}")
    message(STATUS "Uncrustify found: enabling in git hook")
    execute_process(COMMAND ${GIT_EXECUTABLE} config uncrustify.path "${Uncrustify_EXECUTABLE}"
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    execute_process(COMMAND ${GIT_EXECUTABLE} config uncrustify.conf "${Uncrustify_CONFIG}"
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    execute_process( COMMAND ${GIT_EXECUTABLE} config uncrustify.enabled true
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
  else()
    if(NOT EXISTS "${Uncrustify_EXECUTABLE}")
      message(STATUS "Uncrustify executable not found: disabling in git hook")
      execute_process(COMMAND ${GIT_EXECUTABLE} config --unset uncrustify.path
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    endif()
    if(NOT EXISTS "${Uncrustify_CONFIG}")
      message(STATUS "Uncrustify config file not found: disabling in git hook")
      execute_process(COMMAND ${GIT_EXECUTABLE} config --unset uncrustify.conf
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    endif()
    execute_process(COMMAND ${GIT_EXECUTABLE} config uncrustify.enabled false
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
  endif()
endif()
