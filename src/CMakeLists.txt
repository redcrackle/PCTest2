cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCTest2)
find_package(PCL 1.3 REQUIRED COMPONENTS common io sample_consensus segmentation)
include_directories(${PCL_INCLUDE_DIRS} include Shader gl3w/include GLRenderer Camera Utils RMFE Histogram Histogram2D)
add_subdirectory(Shader)
add_subdirectory(GLRenderer)
add_subdirectory(gl3w)
add_subdirectory(Camera)
add_subdirectory(Utils)
add_subdirectory(RMFE)
add_subdirectory(Histogram)
add_subdirectory(Histogram2D)

link_directories(${PCL_LIBRARY_DIRS} ../src/lib/mac "/System/Library/Frameworks/OpenGL.framework/Libraries")
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DOSX -Wno-deprecated-register -Wno-deprecated-declarations -Wno-shift-op-parentheses -Wno-header-guard -fvisibility-inlines-hidden)
set(CMAKE_BUILD_TYPE Debug)

add_definitions(-DUNICODE -D_UNICODE)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

#set(CMAKE_PREFIX_PATH /Users/neeravbm/OpenCV/opencv)
#find_package(OpenCV REQUIRED)

#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIRS})

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
endif()

add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} "-framework Foundation" GL glfw3 m stdc++ freeimage Utils Shader GLRenderer gl3w Camera RMFE Histogram Histogram2D)