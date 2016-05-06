# Comments by Alexis:
# VRPN has a FindVRPN, Findquatlib, FindLibusb1, FindHIDAPI in cmake folder:
# - FindLibusb1 finds the include but not the lib. Updating it from usb-1.0 to
#   libusb-1.0 fixes the issue.
# - Findquatlib does not find lib or include. Updating lib from quat.lib/libquat.a
#   to quat and quatd resolved the library. No way to resolve the include since
#   QUATLIB_ROOT_DIR is not defined in this scope.
# - FindHIDAPI does not find lib or include. Using submodules/hidapi.cmake
#   instead would help finding the good variables but requires more includes and
#   to be in VRPN scope.
# - FindVRPN uses Findquatlib and FindLibusb1 therefore fails. It also does not
#   look for HIDAPI.

#-----------------------------------------------------------------------------
# Find path
#-----------------------------------------------------------------------------
find_path(VRPN_INCLUDE_DIR
  NAMES
    vrpn_Configure.h
    )
mark_as_advanced(VRPN_INCLUDE_DIR)

find_path(LIBUSB1_INCLUDE_DIR
  NAMES
    libusb.h
  )
mark_as_advanced(LIBUSB1_INCLUDE_DIR)

#-----------------------------------------------------------------------------
# Set up include dirs
#-----------------------------------------------------------------------------
list(APPEND VRPN_INCLUDE_DIRS
  ${VRPN_INCLUDE_DIR}
  ${VRPN_INCLUDE_DIR}/quat
  ${VRPN_INCLUDE_DIR}/atmellib
  ${LIBUSB1_INCLUDE_DIR}
  )
message(STATUS "VRPN_INCLUDE_DIRS : ${VRPN_INCLUDE_DIRS}")

#-----------------------------------------------------------------------------
# Find library
#-----------------------------------------------------------------------------
find_library(VRPN_LIBRARY
  NAMES
    vrpnserver
    vrpnserverd
  )
mark_as_advanced(VRPN_LIBRARY)

find_library(QUAT_LIBRARY
  NAMES
    quat
    quatd
  )
mark_as_advanced(QUAT_LIBRARY)

#works on windows, but sounds like it is needed only on linux, check vrpn/submodules/hidapi.cmake
find_library(LIBUSB1_LIBRARY
  NAMES
    libusb-1.0
  )
mark_as_advanced(LIBUSB1_LIBRARY)

#windows only, libusb1 sounds enough on unix, check vrpn/submodules/hidapi.cmake
if(WIN32)
  find_library(HIDAPI_LIBRARY
    NAMES
      setupapi
    )
  mark_as_advanced(HIDAPI_LIBRARY)
endif()

#-----------------------------------------------------------------------------
# Set up libraries
#-----------------------------------------------------------------------------
set(VRPN_LIBRARIES
  ${VRPN_LIBRARY}
  ${QUAT_LIBRARY}
  ${LIBUSB1_LIBRARY}
  ${HIDAPI_LIBRARY}
  )
message(STATUS "VRPN_LIBRARIES : ${VRPN_LIBRARIES}")

#-----------------------------------------------------------------------------
# Find package
#-----------------------------------------------------------------------------
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VRPN
  REQUIRED_VARS
    VRPN_INCLUDE_DIRS
    VRPN_LIBRARIES)

#-----------------------------------------------------------------------------
# If missing target, create it
#-----------------------------------------------------------------------------

if(VRPN_FOUND AND NOT TARGET VRPN)
  add_library(VRPN INTERFACE IMPORTED)
  set_target_properties(VRPN PROPERTIES
    INTERFACE_LINK_LIBRARIES "${VRPN_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${VRPN_INCLUDE_DIRS}"
  )
endif()
