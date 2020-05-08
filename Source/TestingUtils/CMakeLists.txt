#-----------------------------------------------------------------------------
# Create target
#-----------------------------------------------------------------------------
if( ${PROJECT_NAME}_BUILD_TESTING )
  include(imstkAddLibrary)
  imstk_add_library( TestingUtils
    DEPENDS
      GTest::gtest
    )
endif()
