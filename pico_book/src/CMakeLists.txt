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
    ws2812.cpp
    led.cpp
    )

# 为主目标生成 PIO 头文件
pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})



# 包含头文件目录（如果需要）
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}     # 包含ws2812.h头文件
    )

# 链接所需的库
target_link_libraries(${TARGET_NAME} PRIVATE
    pico_stdlib           # Pico 标准库
    hardware_uart         # UART 硬件库
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4 # 使用 Heap4 实现
    hardware_i2c          # I2C 硬件库
    hardware_pio          # PIO 硬件库
    hardware_dma           # DMA 硬件库
    )
    
# 添加额外的输出（如 UF2 文件）
pico_add_extra_outputs(${TARGET_NAME})
