cmake_minimum_required (VERSION 2.8)
project (rfm22-wrapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories (
	${PROJECT_SOURCE_DIR}/include
)

add_subdirectory (src)

add_executable (rfm95-client rfm95-client.cpp)
target_link_libraries (rfm95-client rfm95)

add_executable (rfm95-server rfm95-server.cpp)
target_link_libraries (rfm95-server rfm95)

