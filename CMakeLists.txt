# CMake integration by Christian Frisson

project(haply-api-cpp)

cmake_minimum_required(VERSION 3.0.0)

option(USE_DEBUG "Use Debug" ON)
option(BUILD_DOC "Build documentation" OFF)

if(USE_DEBUG)
        message( "\nConfigured for Debug Build")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_DEBUG -DTI_DEBUG=1")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_DEBUG -DTI_DEBUG=1")
        if(NOT WIN32)
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
        endif()
        set(CMAKE_BUILD_TYPE "Debug")
        set(CMAKE_VERBOSE_MAKEFILE ON)
else(USE_DEBUG)
        message( "\nConfigured for Release Build")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTI_DEBUG=0 -O3")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTI_DEBUG=0 -O3")
        set(CMAKE_BUILD_TYPE "MinSizeRel")#Debug|Release|RelWithDebInfo|MinSizeRel
        set(CMAKE_VERBOSE_MAKEFILE ON)
        if(APPLE)
                set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/release")
        endif()
endif(USE_DEBUG)

if(NOT CMAKE_VERSION VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 11)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

# Core library
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)
set(SRC
src/Haply/Actuator.cpp
src/Haply/Board.cpp
src/Haply/Device.cpp
src/Haply/Pantograph.cpp
src/Haply/Pwm.cpp
src/Haply/Sensor.cpp
src/Haply/SimpleActuatorMech.cpp
)
set(HDR
src/Haply/Actuator.h
src/Haply/Board.h
src/Haply/Device.h
src/Haply/Mechanisms.h
src/Haply/Pantograph.h
src/Haply/Pwm.h
src/Haply/Sensor.h
src/Haply/Serial.h
src/Haply/SimpleActuatorMech.h
src/Haply/Types.h
)

# Use our fork of William Woodall's cross-platform serial port library as serial implementation
include_directories(external/github-HaplyHaptics-serial/include)
add_subdirectory(external/github-HaplyHaptics-serial)
list(APPEND SRC src/Wjwwood/WjwwoodSerial.cpp)
list(APPEND HDR src/Wjwwood/WjwwoodSerial.h)

# Add haply-api-cpp library
add_library(${PROJECT_NAME} ${SRC} ${HDR})
target_link_libraries(${PROJECT_NAME} serial)

# Add simple example for log inspection
add_executable(${PROJECT_NAME}-example example/main.cpp)
target_link_libraries(${PROJECT_NAME}-example ${PROJECT_NAME})

# Documentation
if(BUILD_DOC)  
# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")
    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen needs to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
endif()
