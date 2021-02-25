cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(BORESIGHT)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# add the TestFunction library
add_subdirectory(Func)

add_executable(boresight_calibration boresight_main.cpp)

target_link_libraries(boresight_calibration ${PCL_LIBRARIES} FunctionLibrary)

include_directories(boresight_calibration ${PCL_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/Func")
