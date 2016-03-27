add_library(Utils Utils.cpp)

set(CMAKE_PREFIX_PATH ${CMAKE_MODULE_PATH} /Users/neeravbm/OpenCV/build/share/OpenCV)
find_package(OpenCV REQUIRED core highgui)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
endif()

set(CMAKE_BUILD_TYPE Debug)

add_definitions(-DUNICODE -D_UNICODE)

target_link_libraries(Utils ${OpenCV_LIBS})