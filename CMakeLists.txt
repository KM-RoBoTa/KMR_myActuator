cmake_minimum_required(VERSION 3.20)
project(KMR_myActuator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(${PROJECT_NAME} 
            source/writer.cpp
            source/listener.cpp
            source/motor_handler.cpp
            source/utils.cpp
            source/p1_writer.cpp
            #source/p2_parser.cpp
            source/p1_parser.cpp
            source/p2_parser.cpp
            )

# Directories containing header files
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Locations of the used libraries
#target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/lib)

# Generate Docs
option(BUILD_DOCS "Generate Docs" ON)
if(BUILD_DOCS)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/docs)  # path to docs' cmakelists
endif()
