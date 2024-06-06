#cmake最小需要版本
cmake_minimum_required(VERSION 3.5)
#项目名字
project(RM_CMAKE_EXAMPLE)
#包含原程序,即把给定目录下的源程序复制给变量DIR_SRC
aux_source_directory(DIR_SRC ./)
INCLUDE_DIRECTORIES(./linux-x86/include)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_library(RM_API libRM_API ./linux-x86)

find_package(Threads REQUIRED)

find_package(mc_rtc REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/sanitizers/cmake" ${CMAKE_MODULE_PATH})
find_package(Sanitizers)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/linux-x86/lib)
#生成程序
add_executable(rm_camke_example main.cpp)
add_sanitizers(rm_camke_example)
target_link_libraries(rm_camke_example PUBLIC Threads::Threads mc_rtc::mc_control libRM_Base.so)

if(UNIX)
  target_compile_options(rm_camke_example PUBLIC -pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings )
endif()
