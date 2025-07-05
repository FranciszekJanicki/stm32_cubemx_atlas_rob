add_subdirectory(common)
add_subdirectory(system_task)
add_subdirectory(joints_task)
add_subdirectory(kinematics_task)
add_subdirectory(uart_task)
add_subdirectory(packet_task)

add_library(atlas_rob INTERFACE)

target_link_libraries(atlas_rob INTERFACE 
    common
    system_task
    joints_task
    kinematics_task
    uart_task
    packet_task
)