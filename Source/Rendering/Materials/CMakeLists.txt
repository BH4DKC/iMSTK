#-----------------------------------------------------------------------------
# Create target
#-----------------------------------------------------------------------------
include(imstkAddLibrary)
imstk_add_library( Materials
  DEPENDS
    Common
    ${VTK_LIBRARIES}
  )

#-----------------------------------------------------------------------------
# Testing
#-----------------------------------------------------------------------------
if( ${PROJECT_NAME}_BUILD_TESTING )
  include(imstkAddTest)
  imstk_add_test( Materials )
endif()
