cmake_minimum_required(VERSION 3.18)
project(ProjectName)

find_package(Theia REQUIRED)
include_directories(${THEIA_INCLUDE_DIRS})
#target_link_libraries(my_library ${THEIA_LIBRARIES})

add_executable(target temperature_merge.cpp color_conversion.cpp)
target_link_libraries(target PUBLIC ${THEIA_LIBRARIES})


