cmake_minimum_required(VERSION 3.20)
project(set_id)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_executable(${PROJECT_NAME} 
            setID.cpp
            )
