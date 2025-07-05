add_subdirectory(joint_task)

add_library(joints_task STATIC)

target_sources(joints_task PRIVATE 
    joints_task.c
    joints_manager.c
)

target_include_directories(joints_task PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(joints_task PUBLIC
    joint_task
    common
    stm32cubemx
)

target_compile_options(joints_task PUBLIC
    -std=c23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -Wformat=2
    -Wformat-security
    -Wmissing-prototypes
    -Wmissing-declarations
    -Wstrict-prototypes
    -Wold-style-definition
    -Wundef
    -Wvla
    -Wpointer-arith
    -Wstrict-aliasing=2
)