cmake_minimum_required(VERSION 3.5)
project(afoo)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# Find all packages in the src directory and add them as subdirectories
file(GLOB_RECURSE PACKAGE_CMAKELISTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */CMakeLists.txt)

foreach(package_cmakelists ${PACKAGE_CMAKELISTS})
    get_filename_component(package_dir ${package_cmakelists} DIRECTORY)
    if(NOT ${package_dir} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
        add_subdirectory(${package_dir})
    endif()
endforeach()

ament_package()
