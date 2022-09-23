/*!
 * @file spi_ads.h
 * @brief spi_ads.h detailed description for spi_ads.cpp
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @version V1.0
 * @date 2021-11-10
 */
#pragma once

#include <unistd.h>
#include <sys/param.h>
#include <string.h>
#include <math.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ads_link.h"

#include "hal/gpio_ll.h"

#include "DFRobot_queue.h"

#define SPI_SLAVE_TRANSFER_SIZE 512

#define ADS_WORD_LENGTH 3   // ÿ������3���ֽ�

// ADS ���ź�
#define ADS_HOST    SPI3_HOST
// #define DMA_CHAN    ADS_HOST

#define PIN_NUM_MISO   37   // 37 5
#define PIN_NUM_MOSI   35   // 35 2
#define PIN_NUM_CLK    36   // 36 3
#define PIN_NUM_CS     34   // 34 4
#define RESET          21
#define DRDY_PIN       18

#define ADS_BUSY_TIMEOUT_MS  5

#define ADS_CLK_FREQ         (60*1000*1000)
#define ADS_INPUT_DELAY_NS   (1000000000/8000000/2+20)

#define ADS_ADDR_MASK        0x3F
#define ADS_LENGTH_MASK      0x7F

#define ADS_CMD_NULL         0x0000   ///< No operation
#define ADS_CMD_RESET        0x0011   ///< Reset the device
#define ADS_CMD_STANDBY      0x0022   ///< Place the device into standby mode
#define ADS_CMD_WAKEUP       0x0033   ///< Wake the device from standby mode to conversion mode
#define ADS_CMD_LOCK         0x0555   ///< Lock the interface such that only the NULL, UNLOCK, and RREG commands are valid
#define ADS_CMD_UNLOCK       0x0655   ///< Unlock the interface after the interface is locked
#define ADS_CMD_RREG         0xA000   ///< Read nnn nnnn plus 1 registers beginning at address a aaaa a (101a aaaa annn nnnn)
#define ADS_CMD_WREG         0x6000   ///< Write nnn nnnn plus 1 registers beginning at address a aaaa a (011a aaaa annn nnnn)

#define ADS_CONCAT_TWO_BYTES(msb, lsb)                (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data
#define ADS_CONCAT_THREE_BYTES(msb, lsb, llsb)        (((uint32_t)msb << 24) | ((uint32_t)lsb << 16) | ((uint32_t)llsb << 8))   ///< Macro combines three 8-bit data into one 16-bit data
#define ADS_CONCAT_FOUR_BYTES(mmsb, msb, lsb, llsb)   (((uint32_t)mmsb << 24) | ((uint32_t)msb << 16) | ((uint32_t)lsb << 8) | (uint32_t)llsb)   ///< Macro combines four 8-bit data into one 16-bit data


/// �豸���ú�ָʾ��(ֻ���Ĵ���)
#define ADS_ID_REG             0x00   ///< ID Register (Address = 00h) [reset = 28xxh]
#define ADS_STATUS_REG         0x01   ///< STATUS Register (Address = 01h) [reset = 0500h]

/// ��ͨ����ȫ������
#define ADS_MODE_REG           0x02   ///< MODE Register (Address = 02h) [reset = 0510h]
#define ADS_CLOCK_REG          0x03   ///< CLOCK Register (Address = 03h) [reset = FF0Eh]
#define ADS_GAIN1_REG          0x04   ///< GAIN1 Register (Address = 4h) [reset = 0000h]
#define ADS_GAIN2_REG          0x05   ///< GAIN2 Register (Address = 5h) [reset = 0000h]
#define ADS_CFG_REG            0x06   ///< CFG Register (Address = 06h) [reset = 0600h]
#define ADS_THRSHLD_MSB_REG    0x07   ///< THRSHLD_MSB Register (Address = 07h) [reset = 0000h]
#define ADS_THRSHLD_LSB_REG    0x08   ///< THRSHLD_LSB Register (Address = 08h) [reset = 0000h]

/// �ض�ͨ������
#define ADS_CH0_CFG_REG        0x09   ///< CH0_CFG Register (Address = 09h) [reset = 0000h]
#define ADS_CH0_OCAL_MSB_REG   0x0A   ///< CH0_OCAL_MSB Register (Address = 0Ah) [reset = 0000h]
#define ADS_CH0_OCAL_LSB_REG   0x0B   ///< CH0_OCAL_LSB Register (Address = 0Bh) [reset = 0000h]
#define ADS_CH0_GCAL_MSB_REG   0x0C   ///< CH0_GCAL_MSB Register (Address = 0Ch) [reset = 8000h]
#define ADS_CH0_GCAL_LSB_REG   0x0D   ///< CH0_GCAL_LSB Register (Address = 0Dh) [reset = 0000h]
#define ADS_CH1_CFG_REG        0x0E   ///< CH1_CFG Register (Address = 0Eh) [reset = 0000h]
#define ADS_CH1_OCAL_MSB_REG   0x0F   ///< CH1_OCAL_MSB Register (Address = 0Fh) [reset = 0000h]
#define ADS_CH1_OCAL_LSB_REG   0x10   ///< CH1_OCAL_LSB Register (Address = 10h) [reset = 0000h]
#define ADS_CH1_GCAL_MSB_REG   0x11   ///< CH1_GCAL_MSB Register (Address = 11h) [reset = 8000h]
#define ADS_CH1_GCAL_LSB_REG   0x12   ///< CH1_GCAL_LSB Register (Address = 12h) [reset = 0000h]
#define ADS_CH2_CFG_REG        0x13   ///< CH2_CFG Register (Address = 13h) [reset = 0000h]
#define ADS_CH2_OCAL_MSB_REG   0x14   ///< CH2_OCAL_MSB Register (Address = 14h) [reset = 0000h]
#define ADS_CH2_OCAL_LSB_REG   0x15   ///< CH2_OCAL_LSB Register (Address = 15h) [reset = 0000h]
#define ADS_CH2_GCAL_MSB_REG   0x16   ///< CH2_GCAL_MSB Register (Address = 16h) [reset = 8000h]
#define ADS_CH2_GCAL_LSB_REG   0x17   ///< CH2_GCAL_LSB Register (Address = 17h) [reset = 0000h]
#define ADS_CH3_CFG_REG        0x18   ///< CH3_CFG Register (Address = 18h) [reset = 0000h]
#define ADS_CH3_OCAL_MSB_REG   0x19   ///< CH3_OCAL_MSB Register (Address = 19h) [reset = 0000h]
#define ADS_CH3_OCAL_LSB_REG   0x1A   ///< CH3_OCAL_LSB Register (Address = 1Ah) [reset = 0000h]
#define ADS_CH3_GCAL_MSB_REG   0x1B   ///< CH3_GCAL_MSB Register (Address = 1Bh) [reset = 8000h]
#define ADS_CH3_GCAL_LSB_REG   0x1C   ///< CH3_GCAL_LSB Register (Address = 1Ch) [reset = 0000h]
#define ADS_CH4_CFG_REG        0x1D   ///< CH4_CFG Register (Address = 1Dh) [reset = 0000h]
#define ADS_CH4_OCAL_MSB_REG   0x1E   ///< CH4_OCAL_MSB Register (Address = 1Eh) [reset = 0000h]
#define ADS_CH4_OCAL_LSB_REG   0x1F   ///< CH4_OCAL_LSB Register (Address = 1Fh) [reset = 0000h]
#define ADS_CH4_GCAL_MSB_REG   0x20   ///< CH4_GCAL_MSB Register (Address = 20h) [reset = 8000h]
#define ADS_CH4_GCAL_LSB_REG   0x21   ///< CH4_GCAL_LSB Register (Address = 21h) [reset = 0000h]
#define ADS_CH5_CFG_REG        0x22   ///< CH5_CFG Register (Address = 22h) [reset = 0000h]
#define ADS_CH5_OCAL_MSB_REG   0x23   ///< CH5_OCAL_MSB Register (Address = 23h) [reset = 0000h]
#define ADS_CH5_OCAL_LSB_REG   0x24   ///< CH5_OCAL_LSB Register (Address = 24h) [reset = 0000h]
#define ADS_CH5_GCAL_MSB_REG   0x25   ///< CH5_GCAL_MSB Register (Address = 25h) [reset = 8000h]
#define ADS_CH5_GCAL_LSB_REG   0x26   ///< CH5_GCAL_LSB Register (Address = 26h) [reset = 0000h]
#define ADS_CH6_CFG_REG        0x27   ///< CH6_CFG Register (Address = 27h) [reset = 0000h]
#define ADS_CH6_OCAL_MSB_REG   0x28   ///< CH6_OCAL_MSB Register (Address = 28h) [reset = 0000h]
#define ADS_CH6_OCAL_LSB_REG   0x29   ///< CH6_OCAL_LSB Register (Address = 29h) [reset = 0000h]
#define ADS_CH6_GCAL_MSB_REG   0x2A   ///< CH6_GCAL_MSB Register (Address = 2Ah) [reset = 8000h]
#define ADS_CH6_GCAL_LSB_REG   0x2B   ///< CH6_GCAL_LSB Register (Address = 2Bh) [reset = 0000h]
#define ADS_CH7_CFG_REG        0x2C   ///< CH7_CFG Register (Address = 2Ch) [reset = 0000h]
#define ADS_CH7_OCAL_MSB_REG   0x2D   ///< CH7_OCAL_MSB Register (Address = 2Dh) [reset = 0000h]
#define ADS_CH7_OCAL_LSB_REG   0x2E   ///< CH7_OCAL_LSB Register (Address = 2Eh) [reset = 0000h]
#define ADS_CH7_GCAL_MSB_REG   0x2F   ///< CH7_GCAL_MSB Register (Address = 2Fh) [reset = 8000h]
#define ADS_CH7_GCAL_LSB_REG   0x30   ///< CH7_GCAL_LSB Register (Address = 30h) [reset = 0000h]

#define ADS_REGMAP_CRC_REG     0x3E   ///< REGMAP_CRC Register (Address = 3Eh) [reset = 0000h]
#define ADS_RESERVED_REG       0x3F   ///< RESERVED Register (Address = 3Fh) [reset = 0000h]

/// ͨ��Э��ṹ��?
typedef struct {
    uint8_t cmd[2];
    uint8_t dummy_byte1;
    uint8_t data[24];
    uint8_t crc[2];
    uint8_t dummy_byte2;
} ads_communication_t;

/// spi_ads������
typedef struct {
    spi_host_device_t host; ///< ʹ�õ�SPI����, �ڵ���' spi_ads_init() '֮ǰ����
    gpio_num_t cs_io;       ///< CS gpio��, �ڵ���' spi_ads_init() '֮ǰ����
    gpio_num_t miso_io;     ///< MISO gpio��, �ڵ���' spi_ads_init() '֮ǰ����
    gpio_num_t drdy_io;     ///< MISO gpio��, �ڵ���' spi_ads_init() '֮ǰ����
    gpio_num_t reset_io;     ///< MISO gpio��, �ڵ���' spi_ads_init() '֮ǰ����
    bool intr_used;         ///< �ڵȴ�д����ʱ, ��ʹ����ѯ�����жϡ��ڵ���' spi_ads_init() '֮ǰ���á�
} ads_config_t;

/// spi_ads��������(config��data)
struct ads_context_t{
    ads_config_t cfg;           ///< �ɵ���������
    spi_device_handle_t spi;    ///< SPI�豸���?
    xSemaphoreHandle ready_sem; ///< ׼���źŵ��ź�
};

typedef struct ads_context_t ads_context_t;
typedef struct ads_context_t* ads_handle_t;

//����Щͨ����ʹ��
extern uint8_t enable_channel;
//�ж��ٸ�ͨ����ʹ��
extern uint8_t channel_conut;

/**
 * @brief ��ʼ��Ӳ��
 * @return
 *      - ESP_ERR_INVALID_ARG   if configuration is invalid
 *      - ESP_ERR_INVALID_STATE if host already is in use
 *      - ESP_ERR_NOT_FOUND     if there is no available DMA channel
 *      - ESP_ERR_NO_MEM        if out of memory
 *      - ESP_OK                on success
 *      - ��������' spi_bus_add_device() '��' gpio_isr_handler_add() '����������ֵ
 */
esp_err_t spi_ads_init(void);

/**
 * @brief �ͷ�ADSʹ�õ���Դ
 * @return ���� ESP_OK
 */
esp_err_t spi_ads_deinit(void);

/**
 * @brief ͨ��SPI���߶�ȡ�Ĵ���ֵ
 * @param addr ���Ĵ����ĵ�ַ
 * @param read_buf �����������������?
 * @param size ���Ĵ����ĸ���
 * @return ��' spi_device_polling_transmit() '����ֵ
 */
esp_err_t ads_read_reg(uint8_t addr, uint8_t* read_buf, uint8_t size);

/**
 * @brief ͨ��SPI����д��Ĵ����?
 * @param addr д�Ĵ����ĵ�ַ
 * @param read_buf Ҫд�������?
 * @param size д�Ĵ����ĸ���
 * @return
 *  - ESP_OK: �ɹ�
 *  - ESP_ERR_TIMEOUT: ���ADS�����ڹ���й�?��ʱ��ǰ׼���á��������ζ�����Ӳ���ȷ��?
 *  - ��������' spi_device_acquire_bus() '��' spi_device_polling_transmit() '����������ֵ��
 */
esp_err_t ads_write_reg(uint8_t addr, const uint8_t* write_buf, uint8_t size);

/**
 * @brief ��ȡadc�ɼ�ģ��������������?, ��������Ϣ����һ��ȫ�ֵ�
 * @n adc_module_cfg_table�У���ʱ�洢������4��������Ϣ, �������ܻ����Ӹ��ࣩ
 * @return bool ����
 *  - true ��ȡ������Ϣ�ɹ�
 *  - False ��ȡ������Ϣʧ��
 */
bool get_adc_module_cfg_table(void);

/**
 * @brief ����adcģ��Ĺ���������?
 * @param osr_multiple ����������, ������ģʽ����:
 * @n osr_128, osr_256, osr_512, osr_1024(default),
 * @n osr_2048, osr_4096, osr_8192, osr_16256
 * @return bool ����
 *  - true ����������Ϣ�ɹ�
 *  - False ����������Ϣʧ��
 */
bool set_adc_module_cfg_osr(osr_type osr_multiple);

/**
 * @brief ����adcģ��ķֱ���?
 * @param multiple ģ��ķֱ���?(����ģʽ), ������ģʽ����:
 * @n very_low_power, low_power, high_resolution(default)
 * @return bool ����
 *  - true ����������Ϣ�ɹ�
 *  - False ����������Ϣʧ��
 */
bool set_adc_module_cfg_resolution_rate(PWR_type resolution_rate);

/**
 * @brief ����adcģ������汶��?
 * @param channel Ҫ�������汶����ͨ��, �����õ�ͨ��Ϊ: 0~7
 * @param gain_multiple ���汶��, ����������ģʽ����:
 * @n PGA_1(default), PGA_2, PGA_4, PGA_8, PGA_16, PGA_32, PGA_64, PGA_128
 * @return bool ����
 *  - true ����������Ϣ�ɹ�
 *  - False ����������Ϣʧ��
 */
bool set_adc_module_cfg_gain(uint8_t channel, PGA_type gain_multiple);

/**
 * @brief ����adcģ����ļ����?��ʹ��
 * @param adc_channel һ��8λ����,
 * @n ÿһλ��ʾһ��ͨ��, ��λ1��ʾʹ��, 0��ʾʧ��,
 * @n ����, 0x55,��ʾʹ��ͨ��0, 2, 4, 6
 * @return bool ����
 *  - true ����������Ϣ�ɹ�
 *  - False ����������Ϣʧ��
 */
bool enable_adc_channel(uint8_t adc_channel);

/**
 * @brief ���ݿ�������Щͨ�����ɼ�һ��adc���ݣ����ҽ�����
 * @n ���뵽�����е�����buffer�У�ÿ�βɼ�16��adc����
 * @return
 *  - ESP_OK: �ɹ�
 *  - ESP_ERR_TIMEOUT: ���ADS�����ڹ���й�?��ʱ��ǰ׼����, �������ζ�����Ӳ���ȷ��?
 *  - ESP_ERR_NO_MEM: ��������ź���?(����)ʧ��, ���ٿռ�ʧ��, û�ж���ռ��ˡ�?
 *  - ��������' spi_device_polling_transmit() '����������ֵ
 */
esp_err_t collect_adc_data(void);

/**
 * @brief ��ȡĳ��ͨ����һ���Ѳɼ��õ�adc����
 * @param adc_data ��ȡһ�����ݵĴ�Ż���������С���?48���ֽ�
 * @return
 *  - ESP_OK: �ɹ�
 *  - ESP_ERR_NOT_FOUND: ��ǰ��������û������
 *  - ESP_FAIL: ����������ռ��ͷ�ʧ��?
 */
esp_err_t get_adc_data(uint8_t* adc_data);

esp_err_t test_adc_data(void);
