cmake_minimum_required(VERSION 2.8.3)
project(bezier_curve)

ENABLE_LANGUAGE(CXX)
SET(CMAKE_BUILD_TYPE "Debug")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	rviz_visual_tools
)

###########
## Build ##
###########

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/bezier_curve.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
)

## add test subdirectory
add_subdirectory(test)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
