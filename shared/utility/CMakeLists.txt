# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/)

# This gets all of our source files
FILE(GLOB_RECURSE src "*/**.cpp" "*/**.h")

# Build a library from these files
ADD_LIBRARY(utility ${src})

# Put it in an IDE group for shared
SET_PROPERTY(TARGET utility PROPERTY FOLDER "shared/")
