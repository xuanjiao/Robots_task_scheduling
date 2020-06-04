# - Try to find MYSQLCPPCONN
#
# The following variables are optionally searched for defaults
#  MYSQLCPPCONN_ROOT_DIR:            Base directory where all MYSQLCPPCONN components are found
#
# The following are set after configuration is done:
#  MYSQLCPPCONN_FOUND
#  MYSQLCPPCONN_INCLUDE_DIRS
#  MYSQLCPPCONN_LIBRARIES
#  MYSQLCPPCONN_LIBRARYRARY_DIRS

include(FindPackageHandleStandardArgs)

set(MYSQLCPPCONN_ROOT_DIR "" CACHE PATH "Folder contains mysqlcppconn")

if(WIN32)
    find_path(MYSQLCPPCONN_INCLUDE_DIR cppconn/driver.h
        PATHS ${MYSQLCPPCONN_ROOT_DIR})
else()
    find_path(MYSQLCPPCONN_INCLUDE_DIR cppconn/dirver.h
        PATHS ${MYSQLCPPCONN_ROOT_DIR})
endif()

if(MSVC)
    find_library(MYSQLCPPCONN_LIBRARY_RELEASE mysqlcppconn
        PATHS ${MYSQLCPPCONN_ROOT_DIR}
        PATH_SUFFIXES Release)

    find_library(MYSQLCPPCONN_LIBRARY_DEBUG mysqlcppconn
        PATHS ${MYSQLCPPCONN_ROOT_DIR}
        PATH_SUFFIXES Debug)

    set(MYSQLCPPCONN_LIBRARY optimized ${MYSQLCPPCONN_LIBRARY_RELEASE} debug ${MYSQLCPPCONN_LIBRARY_DEBUG})
else()
    find_library(MYSQLCPPCONN_LIBRARY mysqlcppconn
        PATHS ${MYSQLCPPCONN_ROOT_DIR}
        PATH_SUFFIXES lib lib64)
endif()

find_package_handle_standard_args(MYSQLCPPCONN DEFAULT_MSG MYSQLCPPCONN_INCLUDE_DIR MYSQLCPPCONN_LIBRARY)

if(MYSQLCPPCONN_FOUND)
  set(MYSQLCPPCONN_INCLUDE_DIRS ${MYSQLCPPCONN_INCLUDE_DIR})
  set(MYSQLCPPCONN_LIBRARIES ${MYSQLCPPCONN_LIBRARY})
  message(STATUS "Found mysqlcppconn    (include: ${MYSQLCPPCONN_INCLUDE_DIR}, library: ${MYSQLCPPCONN_LIBRARY})")
  mark_as_advanced(MYSQLCPPCONN_ROOT_DIR MYSQLCPPCONN_LIBRARY_RELEASE MYSQLCPPCONN_LIBRARY_DEBUG
                                 MYSQLCPPCONN_LIBRARY MYSQLCPPCONN_INCLUDE_DIR)
endif()
