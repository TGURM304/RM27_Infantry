cmake_minimum_required(VERSION 3.22)
project(app VERSION 0.0.1 LANGUAGES C CXX ASM)

set(SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/main/main.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/task/chassis/chassis_task.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/task/gimbal/SMC/slidingmodec.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/task/gimbal/gimbal_task.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/task/shoot/shoot_task.cc"
)

add_library(app OBJECT ${SOURCES})
target_include_directories(app
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/task"
)

target_link_libraries(app PUBLIC components bsp m)

set_target_properties(app PROPERTIES
        C_STANDARD 17
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED YES
        POSITION_INDEPENDENT_CODE NO
)
