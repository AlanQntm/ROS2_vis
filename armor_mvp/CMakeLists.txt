cmake_minimum_required(VERSION 3.10)
project(armor_mvp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Release")
  add_compile_options(-O3 -DNDEBUG)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio video)

include_directories(${OpenCV_INCLUDE_DIRS} include)

file(GLOB ARMOR_SRC
  src/*.cpp
)

add_executable(armor_mvp ${ARMOR_SRC} main.cpp)
target_link_libraries(armor_mvp ${OpenCV_LIBS})

install(TARGETS armor_mvp RUNTIME DESTINATION bin)
