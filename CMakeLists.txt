cmake_minimum_required(VERSION 3.25)
project(Serial)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


set(LIBSERIAL_SOURCES src/Serial.cpp)
set(LIBSERIAL_INCLUDE_DIRECTORIES include)

# libSocket.a
add_library(Serial_static STATIC ${LIBSERIAL_SOURCES})
target_include_directories(Serial_static PUBLIC ${LIBSERIAL_INCLUDE_DIRECTORIES})
set_target_properties(
    Serial_static
    PROPERTIES 
    OUTPUT_NAME Serial
)

# libSocket.so
add_library(Serial_shared SHARED ${LIBSERIAL_SOURCES})
target_include_directories(Serial_shared PUBLIC ${LIBSERIAL_INCLUDE_DIRECTORIES})
set_target_properties(
    Serial_shared
    PROPERTIES 
    OUTPUT_NAME Serial
)
