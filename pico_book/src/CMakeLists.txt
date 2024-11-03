# 设置目标名称为 main
set(TARGET_NAME main)

# 添加可执行文件，使用 main.c 作为源文件
add_executable(${TARGET_NAME}
    main.cpp
    Mp3.cpp
    max30102.cpp
    heartRate.cpp
    data_global.c
    irq.c
    ssd1306_i2c.c
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
    hardware_i2c          # I2C 硬件库
    )
    
# 添加额外的输出（如 UF2 文件）
pico_add_extra_outputs(${TARGET_NAME})
