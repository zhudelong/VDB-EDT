cmake_minimum_required(VERSION 2.8.3)
project(openvdb_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/openvdb)

ExternalProject_Add(openvdb_src
  URL https://github.com/AcademySoftwareFoundation/openvdb/archive/v6.1.0.zip
  CMAKE_ARGS -DCMAKE_MODULE_PATH=${CMAKE_MODULE_PATH} -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  UPDATE_COMMAND ""
  PATCH_COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeLists.txt ../openvdb_src/
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} openvdb_src)
target_link_libraries(${PROJECT_NAME} 
  ${CATKIN_DEVEL_PREFIX}/lib/libopenvdb${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()
cs_export(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/
  CFG_EXTRAS openvdb-extras.cmake)
