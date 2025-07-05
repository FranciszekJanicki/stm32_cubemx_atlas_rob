add_library(kinematics_task STATIC)

target_sources(kinematics_task PRIVATE 
    kinematics_task.c
    kinematics_manager.c
)

target_include_directories(kinematics_task PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(kinematics_task PUBLIC
    common
    stm32cubemx
)

target_compile_options(kinematics_task PUBLIC
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