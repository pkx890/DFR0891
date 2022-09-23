/*!
 * @file ads_link.h
 * @brief ads_link.h 关于数据存储链表的定义文件
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @version V1.0
 * @date 2021-11-12
 */
#pragma once

#define SAVE_LIST_DATA_SIZE 1 //数据存储链表每个节点中有效数据的大小
#define LINK_SIZE (SAVE_LIST_DATA_SIZE * 3)

// Power mode selection
typedef enum
{
   very_low_power=0,
   low_power,
   high_resolution,   // default
}PWR_type;

// Modulator oversampling ratio selection
typedef enum
{
   osr_128=0,
   osr_256,
   osr_512,
   osr_1024,   // default
   osr_2048,
   osr_4096,
   osr_8192,
   osr_16256,
}osr_type;

// PGA(Programmable Gain Amplifier) gain selection for channel 3
typedef enum
{
   PGA_1=0,   // default
   PGA_2,
   PGA_4,
   PGA_8,
   PGA_16,
   PGA_32,
   PGA_64,
   PGA_128,
}PGA_type;

/**
 * ADC模块配置信息:
 * ADC模块设置寄存器定义
 * =====================================================================================================
 * adc_module_cfg_table[0] |adc_module_cfg_table[1] |adc_module_cfg_table[2] |adc_module_cfg_table[3] |...
 *    过采样倍数（osr）    |   数据输出速率（odr）  |         分辨率         |     放大器增益倍数     |...
 * =====================================================================================================
 */
//#define TABLE_SIZE 4
//uint8_t adc_module_cfg_table[TABLE_SIZE];
// ADC模块配置信息结构体
typedef struct
{
   PWR_type pwr;
   osr_type osr;
//   float odr;   // 单位 SPS
   PGA_type gain_channel0;
   PGA_type gain_channel1;
   PGA_type gain_channel2;
   PGA_type gain_channel3;
   PGA_type gain_channel4;
   PGA_type gain_channel5;
   PGA_type gain_channel6;
   PGA_type gain_channel7;
}ads_cfg_table_t;

ads_cfg_table_t adc_module_cfg_table;

// 获取ADC数据, 通道x的链表
typedef struct channelxDataLink
{
   // 表示最多可以存储LINK_SIZE个adc数据
   uint8_t data_buf[LINK_SIZE];
   struct channelxDataLink *next;
}channelx_data_link;

channelx_data_link *channel_data_head_link_buf[8];

esp_err_t clear_all_adc_data(void);


