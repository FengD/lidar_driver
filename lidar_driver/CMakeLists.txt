cmake_minimum_required(VERSION 2.8)

project(lidar_driver)

find_package(PCL REQUIRED)

# add cxx flags
add_definitions(-std=c++11 -fPIC)
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")

aux_source_directory(src SRCS)
include_directories(
  include
)

add_library(lidar_driver SHARED ${SRCS})
target_link_libraries(lidar_driver
  pthread
  ${PCL_LIBRARIES}
)

add_library(lidar_driver_static SHARED ${SRCS})
target_link_libraries(lidar_driver_static
  pthread
  ${PCL_LIBRARIES}
)

SET(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/build)

IF (BUILD_EXAMPLE)
  add_executable(lidar_driver_example example/main.cpp)

  add_dependencies(lidar_driver_example
    lidar_driver
  )

  target_link_libraries(lidar_driver_example
    lidar_driver
    pthread
    boost_system
    boost_filesystem
    boost_thread
    pcl_io
    pcl_visualization
  )

  install(TARGETS lidar_driver_example DESTINATION ${CMAKE_INSTALL_PREFIX}/example)
ENDIF (BUILD_EXAMPLE)


install(FILES include/driver.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(TARGETS lidar_driver lidar_driver_static
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
