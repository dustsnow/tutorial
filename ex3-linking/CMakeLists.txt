cmake_minimum_required (VERSION 2.6)
project (ex2)

# Include the Eigen folder - it should be in /usr/include/eigen3 - this is similar to -I flag
include_directories(/usr/include/eigen3)
include_directories(/usr/local/include/fcl)


# Include the subfolder
include_directories(subfolder)



# Identify the executable and the source file that we are building (similar to -o command to g++)
add_executable(main main.cpp subfolder/local.cpp)
