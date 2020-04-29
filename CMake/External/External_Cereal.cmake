#-----------------------------------------------------------------------------
# Add External Project
#-----------------------------------------------------------------------------
include(imstkAddExternalProject)
imstk_add_external_project(Cereal
  GIT_REPOSITORY "https://github.com/USCiLab/cereal.git"
  GIT_TAG v1.3.0
  CMAKE_CACHE_ARGS
        -DBUILD_TESTING:BOOL=OFF
  RELATIVE_INCLUDE_PATH ""
  DEPENDENCIES ""
  #VERBOSE
)
if(NOT USE_SYSTEM_Cereal)
  set(Cereal_DIR ${CMAKE_INSTALL_PREFIX}/share/cereal/cmake)
  #message(STATUS "Cereal_DIR : ${Cereal_DIR}")
endif()
