/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief ESP LCD & Touch: AXS15231B
 */

#pragma once

#include "hal/spi_ll.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_panel_vendor.h"

// 指令
#define AXS_LCD_NOP      LCD_CMD_NOP     // 00h 无操作
#define AXS_LCD_SWRESET  LCD_CMD_SWRESET // 01h 软件复位
#define AXS_LCD_SLPIN    LCD_CMD_SLPIN   // 10h 进入睡眠模式
#define AXS_LCD_SLPOUT   LCD_CMD_SLPOUT  // 11h 退出睡眠模式
#define AXS_LCD_PTLON    LCD_CMD_PTLON   // 12h 开启局部显示模式
#define AXS_LCD_NORON    LCD_CMD_NORON   // 13h 关闭局部显示模式（恢复普通模式）
#define AXS_LCD_INVOFF   LCD_CMD_INVOFF  // 20h 关闭显示反转
#define AXS_LCD_INVON    LCD_CMD_INVON   // 21h 开启显示反转
#define AXS_LCD_ALLPOFF  0x22            // 22h 关闭所有像素
#define AXS_LCD_ALLPON   0x23            // 23h 打开所有像素
#define AXS_LCD_DISPOFF  LCD_CMD_DISPOFF // 28h 关闭显示屏
#define AXS_LCD_DISPON   LCD_CMD_DISPON  // 29h 打开显示屏
#define AXS_LCD_TEOFF    LCD_CMD_TEOFF   // 34h 关闭撕裂效果线
#define AXS_LCD_IDMOFF   LCD_CMD_IDMOFF  // 38h 关闭空闲模式
#define AXS_LCD_IDMON    LCD_CMD_IDMON   // 39h 开启空闲模式
// 读取
#define AXS_LCD_RDDID    LCD_CMD_RDDID      // 04h 读取显示屏 ID
#define AXS_LCD_RDNUMED  0x05               // 05h 读取 DSI 错误数量
#define AXS_LCD_RDDST    LCD_CMD_RDDST      // 09h 读取显示屏状态
#define AXS_LCD_RDDPM    LCD_CMD_RDDPM      // 0ah 读取显示屏电源模式
#define AXS_LCD_RDDMADC  LCD_CMD_RDD_MADCTL // 0bh 读取内存数据访问控制
#define AXS_LCD_RDDIPF   LCD_CMD_RDD_COLMOD // 0ch 读取接口像素格式
#define AXS_LCD_RDDIM    LCD_CMD_RDDIM      // 0dh 读取显示屏图像模式
#define AXS_LCD_RDDSM    LCD_CMD_RDDSM      // 0eh 读取显示屏信号模式
#define AXS_LCD_RDDSDR   LCD_CMD_RDDSR      // 0fh 读取显示屏自检结果
#define AXS_LCD_RAMRD    LCD_CMD_RAMRD      // 2eh 内存读取
#define AXS_LCD_RAMRDC   LCD_CMD_RAMRDC     // 3eh 继续内存读取
#define AXS_LCD_RDTESCAN LCD_CMD_GDCAN      // 45h 获取撕裂扫描线
#define AXS_LCD_RDDISBV  LCD_CMD_RDDISBV    // 52h 读取显示屏亮度值
#define AXS_LCD_RDCTRLD  0x54               // 54h 读取控制显示屏
#define AXS_LCD_RDFCHKSU 0xAA               // aah 读取首个校验和
#define AXS_LCD_RDCCHKSU 0xAF               // afh 继续读取校验和
#define AXS_LCD_RDID1    0xDA               // dah 读取 ID1
#define AXS_LCD_RDID2    0xDB               // dbh 读取 ID2
#define AXS_LCD_RDID3    0xDC               // dch 读取 ID3
// 写入
#define AXS_LCD_ALLPFILL 0x24            // 24h 填充所有像素为指定颜色
#define AXS_LCD_GAMSET   LCD_CMD_GAMSET  // 26h 设置伽马曲线
#define AXS_LCD_CASET    LCD_CMD_CASET   // 2ah 设置列地址
#define AXS_LCD_RASET    LCD_CMD_RASET   // 2bh 设置行地址
#define AXS_LCD_RAMWR    LCD_CMD_RAMWR   // 2ch 内存写入
#define AXS_LCD_RAWFILL  0x2F            // 2fh 在指定窗口内填充内存为指定颜色
#define AXS_LCD_PTLAR    LCD_CMD_PTLAR   // 30h 设置局部显示的起始/结束地址
#define AXS_LCD_PTLARC   0x31            // 31h 设置局部显示的列范围
#define AXS_LCD_VSCRDEF  LCD_CMD_VSCRDEF // 33h 定义垂直滚动
#define AXS_LCD_TEON     LCD_CMD_TEON    // 35h 开启撕裂效果线
#define AXS_LCD_MADCTL   LCD_CMD_MADCTL  // 36h 设置内存数据访问控制
#define AXS_LCD_VSCRSADD LCD_CMD_VSCSAD  // 37h 设置垂直滚动起始地址
#define AXS_LCD_IPF      LCD_CMD_COLMOD  // 3ah 设置接口像素格式
#define AXS_LCD_RAMWRC   LCD_CMD_RAMWRC  // 3ch 继续内存写入
#define AXS_LCD_TESCAN   LCD_CMD_STE     // 44h 设置撕裂扫描线
#define AXS_LCD_WRDISBV  LCD_CMD_WRDISBV // 51h 写入显示屏亮度值
#define AXS_LCD_WRCTRLD  0x53            // 53h 写入控制显示屏
#define AXS_LCD_DSTB     0x90            // 90h 进入深度待机模式

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief LCD panel initialization commands.
     *
     */
    typedef struct
    {
        int cmd;               /*<! The specific LCD command */
        const void *data;      /*<! Buffer that holds the command specific data */
        size_t data_bytes;     /*<! Size of `data` in memory, in bytes */
        unsigned int delay_ms; /*<! Delay in milliseconds after this command */
    } axs15231b_lcd_init_cmd_t;

    /**
     * @brief LCD panel vendor configuration.
     *
     * @note  This structure needs to be passed to the `vendor_config` field in `esp_lcd_panel_dev_config_t`.
     *
     */
    typedef struct
    {
        const axs15231b_lcd_init_cmd_t
            *init_cmds;          /*!< Pointer to initialization commands array. Set to NULL if using default commands.
                                  *   The array should be declared as `static const` and positioned outside the function.
                                  *   Please refer to `vendor_specific_init_default` in source file.
                                  */
        uint16_t init_cmds_size; /*<! Number of commands in above array */

        uint16_t width;
        uint16_t height; /*<! Width and height of the panel in pixels */
        struct
        {
            unsigned int use_qspi_interface : 1; /*<! Set to 1 if use QSPI interface, default is SPI interface */
        } flags;
    } axs15231b_vendor_config_t;

    /**
     * @brief Create LCD panel for model AXS15231B
     *
     * @note  Vendor specific initialization can be different between manufacturers, should consult the LCD supplier for
     * initialization sequence code.
     *
     * @param[in] io LCD panel IO handle
     * @param[in] panel_dev_config general panel device configuration
     * @param[out] ret_panel Returned LCD panel handle
     * @return
     *          - ESP_ERR_INVALID_ARG   if parameter is invalid
     *          - ESP_ERR_NO_MEM        if out of memory
     *          - ESP_OK                on success
     */
    esp_err_t esp_lcd_new_panel_axs15231b(const esp_lcd_panel_io_handle_t io,
                                          const esp_lcd_panel_dev_config_t *panel_dev_config,
                                          esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief LCD panel bus configuration structure
 *
 */
#define AXS15231B_PANEL_BUS_I80_CONFIG(dc, wr, clk, d0, d1, d2, d3, d4, d5, d6, d7, b_w, max_trans_sz)                 \
    {                                                                                                                  \
        .dc_gpio_num = dc,                                                                                             \
        .wr_gpio_num = wr,                                                                                             \
        .clk_src = clk,                                                                                                \
        .data_gpio_nums =                                                                                              \
            {                                                                                                          \
                d0,                                                                                                    \
                d1,                                                                                                    \
                d2,                                                                                                    \
                d3,                                                                                                    \
                d4,                                                                                                    \
                d5,                                                                                                    \
                d6,                                                                                                    \
                d7,                                                                                                    \
            },                                                                                                         \
        .bus_width = b_w,                                                                                              \
        .max_transfer_bytes = max_trans_sz,                                                                            \
    }

#define AXS15231B_PANEL_BUS_SPI_CONFIG(sclk, mosi, max_trans_sz)                                                       \
    {                                                                                                                  \
        .sclk_io_num = sclk,                                                                                           \
        .mosi_io_num = mosi,                                                                                           \
        .miso_io_num = -1,                                                                                             \
        .quadhd_io_num = -1,                                                                                           \
        .quadwp_io_num = -1,                                                                                           \
        .max_transfer_sz = max_trans_sz,                                                                               \
    }
#define AXS15231B_PANEL_BUS_QSPI_CONFIG(sclk, d0, d1, d2, d3, max_trans_sz)                                            \
    {                                                                                                                  \
        .data0_io_num = d0,                                                                                            \
        .data1_io_num = d1,                                                                                            \
        .sclk_io_num = sclk,                                                                                           \
        .data2_io_num = d2,                                                                                            \
        .data3_io_num = d3,                                                                                            \
        .max_transfer_sz = max_trans_sz,                                                                               \
    }

/**
 * @brief LCD panel IO configuration structure
 *
 */
#define AXS15231B_PANEL_IO_I80_CONFIG(cs, dc, cb, cb_ctx)                                                              \
    {                                                                                                                  \
        .cs_gpio_num = cs,                                                                                             \
        .pclk_hz = 20 * 1000 * 1000,                                                                                   \
        .on_color_trans_done = cb,                                                                                     \
        .trans_queue_depth = 10,                                                                                       \
        .user_ctx = cb_ctx,                                                                                            \
        .dc_levels =                                                                                                   \
            {                                                                                                          \
                .dc_idle_level = 0,                                                                                    \
                .dc_cmd_level = 0,                                                                                     \
                .dc_dummy_level = 0,                                                                                   \
                .dc_data_level = 1,                                                                                    \
            },                                                                                                         \
        .lcd_cmd_bits = 8,                                                                                             \
        .lcd_param_bits = 8,                                                                                           \
    }

#define AXS15231B_PANEL_IO_SPI_CONFIG(cs, dc, cb, cb_ctx)                                                              \
    {                                                                                                                  \
        .cs_gpio_num = cs,                                                                                             \
        .dc_gpio_num = dc,                                                                                             \
        .spi_mode = 3,                                                                                                 \
        .pclk_hz = 40 * 1000 * 1000,                                                                                   \
        .trans_queue_depth = 10,                                                                                       \
        .on_color_trans_done = cb,                                                                                     \
        .user_ctx = cb_ctx,                                                                                            \
        .lcd_cmd_bits = 8,                                                                                             \
        .lcd_param_bits = 8,                                                                                           \
    }

#define AXS15231B_PANEL_IO_QSPI_CONFIG(cs, cb, cb_ctx)                                                                 \
    {                                                                                                                  \
        .cs_gpio_num = cs,                                                                                             \
        .dc_gpio_num = -1,                                                                                             \
        .spi_mode = 3,                                                                                                 \
        .pclk_hz = 40 * 1000 * 1000,                                                                                   \
        .trans_queue_depth = 10,                                                                                       \
        .on_color_trans_done = cb,                                                                                     \
        .user_ctx = cb_ctx,                                                                                            \
        .lcd_cmd_bits = 32,                                                                                            \
        .lcd_param_bits = 8,                                                                                           \
        .flags =                                                                                                       \
            {                                                                                                          \
                .quad_mode = true,                                                                                     \
            },                                                                                                         \
    }

    /**
     * @brief Create a new AXS15231B1B touch driver
     *
     * @note  The I2C communication should be initialized before use this function.
     *
     * @param io LCD panel IO handle, it should be created by `esp_lcd_new_panel_io_i2c()`
     * @param config Touch panel configuration
     * @param tp Touch panel handle
     * @return
     *      - ESP_OK: on success
     */
    esp_err_t esp_lcd_touch_new_i2c_axs15231b(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config,
                                              esp_lcd_touch_handle_t *tp);

/**
 * @brief I2C address of the AXS15231B controller
 *
 */
#define ESP_LCD_TOUCH_IO_I2C_AXS15231B_ADDRESS (0x3B)

/**
 * @brief Touch IO configuration structure
 *
 */
#define ESP_LCD_TOUCH_IO_I2C_AXS15231B_CONFIG(scl_speed_hz)                                                            \
    {                                                                                                                  \
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_AXS15231B_ADDRESS,                                                            \
        .scl_speed_hz = scl_speed_hz,                                                                                  \
        .control_phase_bytes = 1,                                                                                      \
        .dc_bit_offset = 0,                                                                                            \
        .lcd_cmd_bits = 8,                                                                                             \
        .flags =                                                                                                       \
            {                                                                                                          \
                .disable_control_phase = 1,                                                                            \
            },                                                                                                         \
    }

#ifdef __cplusplus
}
#endif