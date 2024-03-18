cmake_minimum_required(VERSION 3.0.0)
project(RSDataLogger VERSION 0.1.0)

MESSAGE("CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV 3.0 REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(${PROJECT_NAME}_Test ${PROJECT_SOURCE_DIR}/src/Logger.cpp)

target_include_directories(${PROJECT_NAME} 
   PUBLIC ${PROJECT_SOURCE_DIR}
   PUBLIC ${OpenCV_INCLUDE_DIRS}
   PUBLIC ${realsense2_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} 
   PUBLIC ${OpenCV_LIBS}
   PUBLIC ${realsense2_LIBRARY}
)
