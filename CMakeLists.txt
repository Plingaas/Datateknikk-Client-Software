cmake_minimum_required(VERSION 3.29)
project(SLAM)

set(CMAKE_CXX_STANDARD 20)

#set(Open3D_DIR "Open3D")   # Replace with the extracted path
#set(Open3D_LIB_DIR "${Open3D_DIR}/lib")
#set(Open3D_INCLUDE_DIR "${Open3D_DIR}/include")

add_subdirectory(src)
add_executable(main main.cpp)
add_executable(visual visual.cpp)
add_executable(map map.cpp)
add_executable(BresenHam BresenHamTest.cpp)

target_link_libraries(main PUBLIC lib)
target_link_libraries(visual PUBLIC lib)
target_link_libraries(map PUBLIC lib)
target_link_libraries(BresenHam PUBLIC lib)

#target_include_directories(SLAM PRIVATE "${Open3D_INCLUDE_DIR}" "${Open3D_INCLUDE_DIR}/open3d/3rdparty")
#
#if (WIN32)
#    target_link_libraries(SLAM "${CMAKE_SOURCE_DIR}/Open3D/lib/Open3D.lib" lib)
#elseif (UNIX)
#    target_link_libraries(SLAM PRIVATE ${Open3D_LIB_DIR}/libOpen3D.so)
#endif()


file(COPY "data/" DESTINATION "${CMAKE_BINARY_DIR}/data")
#file(COPY "Open3D/bin/Open3D.dll" DESTINATION "${CMAKE_BINARY_DIR}")