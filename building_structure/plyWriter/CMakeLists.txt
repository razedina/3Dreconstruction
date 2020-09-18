cmake_minimum_required(VERSION 3.18)
project(ProjectName)

find_package(Theia REQUIRED)
include_directories(${THEIA_INCLUDE_DIRS})
#target_link_libraries(my_library ${THEIA_LIBRARIES})

add_executable(plyWriter write_ply_file.cpp dbscan.cpp)
target_link_libraries(plyWriter PUBLIC ${THEIA_LIBRARIES})


