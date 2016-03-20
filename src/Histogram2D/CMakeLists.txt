add_library(Histogram2D Histogram2D.cpp)

#include_directories(Shader Utils ../include)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
endif()

set(CMAKE_BUILD_TYPE Debug)

add_definitions(-DUNICODE -D_UNICODE)

target_link_libraries(Histogram2D)
