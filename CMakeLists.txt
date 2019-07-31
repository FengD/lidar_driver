cmake_minimum_required(VERSION 2.8)

project(library)

set(CMAKE_BUILD_TYPE "RELEASE")

IF (ARM_COMPILE)
  include_directories(
    ${CMAKE_FIND_ROOT_PATH}/usr/include
    ${CMAKE_FIND_ROOT_PATH}/usr/include/pcl-1.8
    ${CMAKE_FIND_ROOT_PATH}/usr/include/eigen3
  )

  link_directories(
    ${CMAKE_FIND_ROOT_PATH}/lib
    ${CMAKE_FIND_ROOT_PATH}/usr/lib
  )
ENDIF (ARM_COMPILE)

add_subdirectory(lidar_driver)
