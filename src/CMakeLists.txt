include_directories (${PROJECT_SOURCE_DIR}/include)

add_library (rfm95 SHARED RH_RF95.cpp)
target_link_libraries (rfm95 mraa)
