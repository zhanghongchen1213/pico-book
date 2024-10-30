# 设置目标名称为 main
set(TARGET_NAME main)

# 添加可执行文件，使用 main.c 作为源文件
add_executable(${TARGET_NAME}
    main.c
    Mp3.cpp
    )

# 包含头文件目录（如果需要）
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    )

# 链接所需的库
target_link_libraries(${TARGET_NAME} PRIVATE
    pico_stdlib           # Pico 标准库
    hardware_uart         # UART 硬件库
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4 # 使用 Heap4 实现
    )
    
# 添加额外的输出（如 UF2 文件）
pico_add_extra_outputs(${TARGET_NAME})
