cmake_minimum_required(VERSION 3.0)
project(MinimalOpenCV)

find_package(OpenCV REQUIRED)
find_package(PROJ REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJ_INCLUDE_DIRS})
include_directories(${GeographicLib_INCLUDE_DIRS})

add_executable(main main.cpp)
add_executable(check check_csv.cpp)

add_library(bfp STATIC bfp.cpp)
target_link_libraries(bfp ${PROJ_LIBRARIES})

target_link_libraries(main bfp)
target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(check ${OpenCV_LIBS})
target_link_libraries(check ${GeographicLib_LIBRARIES})
