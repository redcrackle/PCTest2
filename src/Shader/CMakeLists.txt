add_library(Shader Shader.cpp)

add_definitions(-DUNICODE -D_UNICODE)

target_link_libraries(Shader gl3w)