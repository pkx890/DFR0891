/*
                           _ooOoo_
                          o8888888o
                          88" . "88
                          (| -_- |)
                          O\  =  /O
                       ____/`---'\____
                     .'  \\|     |//  `.
                    /  \\|||  :  |||//  \
                   /  _||||| -:- |||||-  \
                   |   | \\\  -  /// |   |
                   | \_|  ''\---/''  |   |
                   \  .-\__  `-`  ___/-. /
                 ___`. .'  /--.--\  `. . __
              ."" '<  `.___\_<|>_/___.'  >'"".
             | | :  `- \`.;`\ _ /`;.`/ - ` : | |
             \  \ `-.   \_ __\ /__ _/   .-` /  /
        ======`-.____`-.___\_____/___.-`____.-'======
                           `=---='
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

                    佛祖保佑       永无BUG

           本模块已经经过开光处理，绝无可能再产生bug
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#include <pthread.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_ads.h"

/*
SPI receiver (slave) example.

This example is supposed to work together with the SPI sender. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. After a transmission has been set up and we're
ready to send/receive data, this code uses a callback to set the handshake pin high. The sender will detect this and start
sending a transaction. As soon as the transaction is done, the line gets set low again.
*/

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#if 0
#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#endif

#if 1
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define GPIO_HANDSHAKE 0
#define GPIO_MOSI 2
#define GPIO_MISO 5
#define GPIO_SCLK 3
#define GPIO_CS 4

#elif CONFIG_IDF_TARGET_ESP32C3
#define GPIO_HANDSHAKE 3
#define GPIO_MOSI 7
#define GPIO_MISO 2
#define GPIO_SCLK 6
#define GPIO_CS 10

#endif // CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#endif

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST HSPI_HOST
#define DMA_CHAN 2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define RCV_HOST SPI2_HOST
#define DMA_CHAN RCV_HOST

#elif defined CONFIG_IDF_TARGET_ESP32C3
#define RCV_HOST SPI2_HOST
#define DMA_CHAN RCV_HOST

#endif

#define MY_TASKDELAY_MS(time) vTaskDelay(time / portTICK_PERIOD_MS)

bool load_spi_data(void);

//命令宏定义
#define CMD_TRANSFER_START 0x01
#define CMD_TRANSFER_END 0x02
#define CMD_DATA_READY 0x03
#define CMD_CHANNEL_CHANGE 0x04

//用于启用adc使能的标志位
bool adc_work_flag = 0;

//有哪些通道被使能
uint8_t enable_channel = 0x01;
//有多少个通道被使能
uint8_t channel_conut = 1;

// spi_slave 缓存空间
uint8_t sendbuf[SPI_SLAVE_TRANSFER_SIZE] = {0};
uint8_t recvbuf[SPI_SLAVE_TRANSFER_SIZE] = {0};

// Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << GPIO_HANDSHAKE));
}

// Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << GPIO_HANDSHAKE));
}

//设置adc最高采样率
void sampling_rate(void)
{
    if (!set_adc_module_cfg_resolution_rate(high_resolution))
    {
        printf("Failed to set adc module resolution!\r\n");
    }else{
        printf("Success to set adc module resolution!\r\n");
    }
    if (!set_adc_module_cfg_osr(osr_128))
    {
        printf("Failed to set the oversampling multiple of the ADC module!\r\n");
    }else{
        printf("Success to set the oversampling multiple of the ADC module!\r\n");
    }
    if (!enable_adc_channel(enable_channel))
    {
        printf("Failed to enable ADC acquisition channel!\r\n");
    }
}

static void spi_slave_init(void)
{
    esp_err_t ret;

    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        //.max_transfer_sz = 512
    };

    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 4096,//使用默认传输大小4096
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb};

    // Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << GPIO_HANDSHAKE)};

    // Configure handshake line as output
    gpio_config(&io_conf);
    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    // Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    
    printf("init status:%d\r\n", ret);
    assert(ret == ESP_OK);
}

void parse_spi_data(uint8_t *spi_buf)
{
    if (spi_buf[0] != 0xf0)
        return;
    switch (spi_buf[1])
    {
        case CMD_TRANSFER_START:
            clearQueue();
            adc_work_flag = 1;
            break;
        case CMD_TRANSFER_END:
            adc_work_flag = 0;
            break;
        case CMD_CHANNEL_CHANGE:
            enable_channel = spi_buf[2];
            enable_adc_channel(enable_channel);
            channel_conut = 0;
            for(int i=0;i<8;i++){
                if((enable_channel & (1<<i))>0)
                {
                    channel_conut++;
                }
            }
            break;
        default:
            break;
    }
}

//将SPI链表中的数据装载到SPI待发送数据区
bool load_spi_data()
{
    uint8_t temp_buf[LINK_SIZE * 8 + 1] = {0}; //数组+1是为了存储通道
    static int count = 0;                         // send_buff计数
    int t_count = 0;                       // temp_buff计数
    esp_err_t err_code = ESP_OK;
    uint8_t channel = 0;
    int data =0;
    channel = 0;
    t_count=0;
    err_code = get_adc_data(temp_buf);
    if (err_code != ESP_OK)
        return false;
    channel = temp_buf[t_count++]; // temp_buf[0]存储的就是这个节点中数据属于几个通道
    // 当不是所有的通道都被使能时，使能的通道可占用非使能通道的SPI传输位置来传输数据，以此来提高传输效率
    for (int i = 0; i < 8; i++)
    {
        if (channel & (0x01 << i))
        {
            sendbuf[count++] = i | 0xe0;
            for (int j = 3; j > 0; j--)
            {
                sendbuf[count++] = temp_buf[t_count];
                data |= temp_buf[t_count]<<(j*8); 
                t_count++;
            }
            if(count>=SPI_SLAVE_TRANSFER_SIZE)
            {
                count = 0;
                return true;
            }
        }
    }
    return false;
}

//sendbuf被装满的时候置位
bool sendbuf_load_flag = false;

//spi slave时间flag，需要事件填充时置位
bool spi_slave_queue_flag = true;


void all_task(void *arg)
{
    spi_slave_transaction_t t;
    spi_transaction_t *ret_trans;
    memset(&t, 0, sizeof(t));
    t.length = SPI_SLAVE_TRANSFER_SIZE * 8;
    memset(sendbuf, 0, sizeof(sendbuf));
    while (1)
    {
        if(adc_work_flag == 1)
        {
            collect_adc_data();
            if(true == load_spi_data())
            {
                sendbuf_load_flag = true;
            }
        }

        memset(recvbuf, 0, SPI_SLAVE_TRANSFER_SIZE);
        t.rx_buffer = recvbuf;

        if(spi_slave_queue_flag == true)
        {
            if(sendbuf_load_flag == true)
            {
                t.tx_buffer = sendbuf;
            }else
            {
                t.tx_buffer = NULL;
            }            
            spi_slave_queue_trans(RCV_HOST, &t, 0);
            spi_slave_queue_flag = false;
        }       
        if(ESP_OK == spi_device_get_trans_result(ads_handle->spi, &ret_trans,0))
        {
            spi_slave_queue_flag == true;
        }
        
        //解析FT4222数据包
        parse_spi_data(recvbuf);
    }
}


// Main application
void app_main(void)
{
    //ads_task()初始化
    spi_ads_init();
    sampling_rate();

    //spi_slave_task()初始化
    spi_slave_init();

    xTaskCreate(all_task, "ads_task", 1024 * 8, NULL, 2, NULL);
}
