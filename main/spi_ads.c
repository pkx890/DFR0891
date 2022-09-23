/*!
 * @file spi_ads.c
 * @brief spi_ads.c Initialize SPI communication, driving ADS131M08
 * @version V1.0
 * @date 2021-11-10
 */
#include "spi_ads.h"
#include "hal/gpio_ll.h"

static const char TAG[] = "ads";
ads_handle_t ads_handle;

#define gpio_set_level  gpio_set_level_patch
static inline esp_err_t gpio_set_level_patch(gpio_num_t gpio_num, uint32_t level)
{
    gpio_ll_set_level(&GPIO, gpio_num, level);
    return ESP_OK;
}

#define gpio_set_output_enable gpio_set_output_enable_patch
static inline esp_err_t gpio_set_output_enable_patch(gpio_num_t gpio_num)
{
    gpio_ll_output_enable(&GPIO, gpio_num);
    return ESP_OK;
}

#define gpio_set_input_enable gpio_set_input_enable_patch
static inline esp_err_t gpio_set_input_enable_patch(gpio_num_t gpio_num)
{
    gpio_ll_input_enable(&GPIO, gpio_num);
    return ESP_OK;
}

static esp_err_t ads_wait_done(void)
{
    usleep(1);
    if (ads_handle->cfg.intr_used) {
        xSemaphoreTake(ads_handle->ready_sem, 0);
        gpio_set_level(ads_handle->cfg.cs_io, 0);
        gpio_intr_enable(ads_handle->cfg.miso_io);

        uint32_t tick_to_wait = MAX(ADS_BUSY_TIMEOUT_MS / portTICK_PERIOD_MS, 2);
        BaseType_t ret = xSemaphoreTake(ads_handle->ready_sem, tick_to_wait);
        gpio_intr_disable(ads_handle->cfg.miso_io);
        gpio_set_level(ads_handle->cfg.cs_io, 1);

        if (ret != pdTRUE) return ESP_ERR_TIMEOUT;
    } else {
        bool timeout = true;
        gpio_set_level(ads_handle->cfg.cs_io, 0);
        for (int i = 0; i < ADS_BUSY_TIMEOUT_MS * 1000; i ++) {
            if (gpio_get_level(ads_handle->cfg.miso_io)) {
                timeout = false;
                break;
            }
            usleep(1);
        }
        gpio_set_level(ads_handle->cfg.cs_io, 1);
        if (timeout) return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((ads_context_t*)t->user)->cfg.cs_io);
    gpio_set_level(((ads_context_t*)t->user)->cfg.cs_io, 1);
}

static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((ads_context_t*)t->user)->cfg.cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((ads_context_t*)t->user)->cfg.cs_io);
}

void ready_rising_isr(void* arg)
{
    ads_context_t* ctx = (ads_context_t*)arg;
    xSemaphoreGive(ctx->ready_sem);
    ESP_EARLY_LOGV(TAG, "ready detected.");
}

esp_err_t spi_ads_deinit(void)
{
    spi_bus_remove_device(ads_handle->spi);
    if (ads_handle->cfg.intr_used) {
        vSemaphoreDelete(ads_handle->ready_sem);
    }
    free(ads_handle);
    return ESP_OK;
}

esp_err_t spi_ads_init(void)
{
    esp_err_t err;
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    err = spi_bus_initialize(ADS_HOST, &buscfg, ADS_HOST);
    if (err!= ESP_OK) return err;

    ads_config_t ads_config = {
        .host = ADS_HOST,
        .cs_io = PIN_NUM_CS,
        .miso_io = PIN_NUM_MISO,
        .drdy_io = DRDY_PIN,
        .reset_io = RESET,
    };

#ifdef CONFIG_EXAMPLE_INTR_USED
    eeprom_config.intr_used = true;
    gpio_install_isr_service(0);
#endif
    if (ads_config.intr_used && ads_config.host == SPI1_HOST) {
        // ESP_LOGE(TAG, "interrupt cannot be used on SPI1 host.");
        return ESP_ERR_INVALID_ARG;
    }

    ads_handle = (ads_context_t*)malloc(sizeof(ads_context_t));
    if (!ads_handle) return ESP_ERR_NO_MEM;

    *ads_handle = (ads_context_t) {
        .cfg = ads_config,
    };
    gpio_set_input_enable(ads_handle->cfg.drdy_io);
    gpio_set_output_enable(ads_handle->cfg.cs_io);
    gpio_set_output_enable(ads_handle->cfg.reset_io);
    spi_device_interface_config_t devcfg={
//        .command_bits = 10,
//        .address_bits = 6,
//        .dummy_bits = 8,
        .clock_speed_hz = ADS_CLK_FREQ,
        .mode = 1,          //SPI mode 1
        .spics_io_num = -1,
        .queue_size = 1,
        // .flags =  SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
        .pre_cb = cs_low,
        .post_cb = cs_high,
        // .input_delay_ns = ADS_INPUT_DELAY_NS, 
    };
    gpio_set_level(ads_handle->cfg.reset_io, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);
    gpio_set_level(ads_handle->cfg.reset_io, 1);

    err = spi_bus_add_device(ads_handle->cfg.host, &devcfg, &ads_handle->spi);
    if  (err != ESP_OK) {
        goto cleanup;
    }

    gpio_set_level(ads_handle->cfg.cs_io, 0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(ads_handle->cfg.cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    if (ads_handle->cfg.intr_used) {
        ads_handle->ready_sem = xSemaphoreCreateBinary();
        if (ads_handle->ready_sem == NULL) {
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        gpio_set_intr_type(ads_handle->cfg.miso_io, GPIO_INTR_POSEDGE);
        err = gpio_isr_handler_add(ads_handle->cfg.miso_io, ready_rising_isr, ads_handle);
        if (err != ESP_OK) {
            goto cleanup;
        }
        gpio_intr_disable(ads_handle->cfg.miso_io);
    }
    return ESP_OK;

cleanup:
    if (ads_handle->spi) {
        spi_bus_remove_device(ads_handle->spi);
        ads_handle->spi = NULL;
    }
    if (ads_handle->ready_sem) {
        vSemaphoreDelete(ads_handle->ready_sem);
        ads_handle->ready_sem = NULL;
    }
    free(ads_handle);
    for(uint8_t i=0; i<8; i++){
        free(channel_data_head_link_buf[i]);
    }
    return err;
}

esp_err_t send_command(uint16_t command)
{
    esp_err_t err;
    spi_transaction_t t = {
        .length = ADS_WORD_LENGTH * 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {(uint8_t)((command & 0xFF00) >> 8), (uint8_t)(command & 0xFF)},
        .user = ads_handle,
    };
    err = spi_device_polling_transmit(ads_handle->spi, &t);
    if (err!= ESP_OK) return err;
    return ESP_OK;
}

esp_err_t read_data_frame(void * data, uint8_t size)
{
    //等待SPI通信完成标志位
    static bool flag123 = true;

    // ads_communication_t data_frame_1;
    esp_err_t err = ESP_FAIL;
    spi_transaction_t t = {
        .length = size * 8,
        .tx_buffer = NULL,
        .rx_buffer = data,
        .user = ads_handle,
    };   
    // while (gpio_get_level(ads_handle->cfg.drdy_io));
    if(gpio_get_level(ads_handle->cfg.drdy_io))
        return ESP_FAIL;
    if(flag123 == true)
    {
        spi_device_queue_trans(ads_handle->spi, &t, portMAX_DELAY);
    }
    spi_transaction_t *ret_trans;
    err =spi_device_get_trans_result(ads_handle->spi, &ret_trans,1);
    if(err!= ESP_OK)
    {
        flag123 = false;
        return ESP_FAIL;
    }
    else
    {
        flag123 = true;
        return ESP_OK;
    }
}

esp_err_t set_read_data_frame(void * data, uint8_t size)
{
    // ads_communication_t data_frame_1;
    esp_err_t err;
    while (gpio_get_level(ads_handle->cfg.drdy_io));
    spi_transaction_t t = {
        .length = size * ADS_WORD_LENGTH * 8,
        .tx_buffer = data,
        .rx_buffer = data,
        .user = ads_handle,
    };
    err = spi_device_polling_transmit(ads_handle->spi, &t);
    if (err!= ESP_OK) return err;
    return ESP_OK;
}

esp_err_t ads_read_reg(uint8_t addr, uint8_t* read_buf, uint8_t size)
{
    esp_err_t err;
    uint16_t cmd = ADS_CMD_RREG | ((addr & 0x3F) << 7) | ((size - 1) & 0x7F);

    err = send_command(cmd);
    
    if (err!= ESP_OK) return err;

    uint8_t buf[10 * ADS_WORD_LENGTH] = {0};
    err = set_read_data_frame(buf, size);

    if (err!= ESP_OK) return err;

    int offset = 0;
    if(size > 1) offset = ADS_WORD_LENGTH;
    for(int i=0; i<(size*ADS_WORD_LENGTH); i++){
        read_buf[i] = buf[i + offset];
    }

    return ESP_OK;
}

esp_err_t ads_write_reg(uint8_t addr, const uint8_t* write_buf, uint8_t size)
{
    esp_err_t err;
    uint16_t temp = ADS_CMD_WREG | ((addr & 0x3F) << 7) | ((size - 1) & 0x7F);
    uint8_t buf[10 * ADS_WORD_LENGTH] = {(uint8_t)((temp & 0xFF00) >> 8), (uint8_t)(temp & 0xFF)};
    for(int i=0; i<(size*ADS_WORD_LENGTH); i++){
        buf[i+3] = write_buf[i];
    }
    spi_transaction_t t = {
        .length = (size + 1) * ADS_WORD_LENGTH * 8,
        .tx_buffer = buf,
        .user = ads_handle,
    };
    err = spi_device_acquire_bus(ads_handle->spi, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }
    err = spi_device_polling_transmit(ads_handle->spi, &t);
    if (err == ESP_OK) {
        err = ads_wait_done();
    }
    spi_device_release_bus(ads_handle->spi);

    uint8_t response[ADS_WORD_LENGTH] = {0};
    err = set_read_data_frame(response, 1);
    if (err!= ESP_OK) return err;
    if (ADS_CONCAT_TWO_BYTES(response[0], response[1]) != (temp & 0xDFFF))
        return ESP_ERR_INVALID_RESPONSE;

    return err;
}

bool get_adc_module_cfg_table(void)
{
    esp_err_t err;
    uint8_t buf[ADS_WORD_LENGTH] = {0};

    err = ads_read_reg(ADS_CLOCK_REG, buf, 1);
    if (err!= ESP_OK) return false;
    adc_module_cfg_table.pwr = (uint8_t)(buf[1] & 0x03);
    adc_module_cfg_table.osr = (uint8_t)((buf[1] & 0x1C) >> 2);

    // adc_module_cfg_table.odr = 8000.0 * pow(2, adc_module_cfg_table.pwr) / pow(2, adc_module_cfg_table.osr);
    err = ads_read_reg(ADS_GAIN1_REG, buf, 1);
    if (err!= ESP_OK) return false;
    adc_module_cfg_table.gain_channel0 = (uint8_t)(buf[1] & 0x07);
    adc_module_cfg_table.gain_channel1 = (uint8_t)((buf[1] >> 4) & 0x07);
    adc_module_cfg_table.gain_channel2 = (uint8_t)(buf[0] & 0x07);
    adc_module_cfg_table.gain_channel3 = (uint8_t)((buf[0] >> 4) & 0x07);
    err = ads_read_reg(ADS_GAIN2_REG, buf, 1);
    if (err!= ESP_OK) return false;
    adc_module_cfg_table.gain_channel4 = (uint8_t)(buf[1] & 0x07);
    adc_module_cfg_table.gain_channel5 = (uint8_t)((buf[1] >> 4) & 0x07);
    adc_module_cfg_table.gain_channel6 = (uint8_t)(buf[0] & 0x07);
    adc_module_cfg_table.gain_channel7 = (uint8_t)((buf[0] >> 4) & 0x07);

    return true;
}

bool set_adc_module_cfg_osr(osr_type osr_multiple)
{
    esp_err_t err;
    uint8_t buf[ADS_WORD_LENGTH] = {0};
    err = ads_read_reg(ADS_CLOCK_REG, buf, 1);
    if (err!= ESP_OK) return false;
    buf[1] = (buf[1] & 0xE3) | (osr_multiple << 2);   
    err = ads_write_reg(ADS_CLOCK_REG, buf, 1);
    if (err!= ESP_OK) return false;

    return true;
}

bool set_adc_module_cfg_resolution_rate(PWR_type resolution_rate)
{
    esp_err_t err;
    uint8_t buf[ADS_WORD_LENGTH] = {0};
    err = ads_read_reg(ADS_CLOCK_REG, buf, 1);
    if (err!= ESP_OK) return false;
    buf[1] = (buf[1] & 0xFC) | resolution_rate;   
    err = ads_write_reg(ADS_CLOCK_REG, buf, 1);
    if (err!= ESP_OK) return false;

    return true;
}

bool set_adc_module_cfg_gain(uint8_t channel, PGA_type gain_multiple)
{
    esp_err_t err;
    uint8_t buf[ADS_WORD_LENGTH] = {0};
    uint8_t reg, offset;
    uint16_t set_data = 0, mask_code = 0xFFFF;

    if (channel < 4){
        reg = ADS_GAIN1_REG;
        offset = channel * 4;
    }else if((channel >= 4) && (channel < 8)){
        reg = ADS_GAIN2_REG;
        offset = (channel - 4) * 4;
    }else{
        return false;
    }
    set_data |= (uint16_t)(gain_multiple << offset);
    mask_code &= (~((uint16_t)(7 << offset)));

    err = ads_read_reg(reg, buf, 1);
    if (err!= ESP_OK) return false;

    buf[0] = (buf[0] & (uint8_t)(mask_code >> 8)) | (uint8_t)(set_data >> 8);
    buf[1] = (buf[1] & (uint8_t)(mask_code & 0xFF)) | (uint8_t)(set_data & 0xFF);
    err = ads_write_reg(reg, buf, 1);
    if (err!= ESP_OK) return false;

    return true;
}

bool enable_adc_channel(uint8_t adc_channel)
{
    esp_err_t err;
    uint8_t buf[ADS_WORD_LENGTH] = {0};
    err = ads_read_reg(ADS_CLOCK_REG, buf, 1);
    if (err!= ESP_OK) return false;

    buf[0] = adc_channel;
    err = ads_write_reg(ADS_CLOCK_REG, buf, 1);
    err = ads_read_reg(ADS_CLOCK_REG, buf, 1);
    if (err!= ESP_OK) return false;
    return true;
}

esp_err_t test_adc_data(void)
{
    esp_err_t err;
    ads_communication_t data_frame;
    int32_t data = 0;
    uint32_t udata = 0;

    memset(&data_frame, 0x00, sizeof(data_frame));
    err = read_data_frame(&data_frame, sizeof(data_frame) / ADS_WORD_LENGTH);
    if (err!= ESP_OK) return err;
    for(uint8_t i=0; i<8; i++){
        udata = ADS_CONCAT_THREE_BYTES(data_frame.data[i*3], data_frame.data[i*3+1], data_frame.data[i*3+2]);
        data = ((int32_t)udata) >> 8;
        if(i==0)
        printf("data-channel-%u = %d, HEX = %#x\r\n", i, data, (udata >> 8));
    }
    return err;
}

esp_err_t collect_adc_data(void)
{
    if (esp_get_free_heap_size()<240000)
    {
        printf("full\r\n");
        return ESP_FAIL;
    }    
    bool status =false;
    esp_err_t err = ESP_OK;
    uint8_t count=0;
    ads_communication_t data_frame;
    err = read_data_frame(&data_frame, sizeof(data_frame));
    if (err != ESP_OK)
        return err;
    uint8_t *temp_buf = (uint8_t *)malloc(channel_conut * 3);
    if (temp_buf == NULL)
    {
        free(temp_buf);
        temp_buf = NULL;
        return ESP_FAIL;
    }
    for (int i = 0; i < 8; i++)
    {    
        if (enable_channel & (0x01 << i))
        {
            for(int j=0;j<3;j++)
            {
                temp_buf[count++] = data_frame.data[i * 3 + j];
            }
        }
    }
    status = cReadEnqueue(temp_buf, channel_conut * 3);
    free(temp_buf);
    temp_buf = NULL;
    if(status == false )
    {
        return ESP_FAIL;
    }
    return err;
}

esp_err_t get_adc_data(uint8_t* adc_data)
{
    if(queue_is_empty()==true)
        return ESP_FAIL;
    struct sReadData *adc_data_buf = cReadDequeue();
    if (adc_data_buf == NULL)
    {
        free(adc_data_buf);
        adc_data_buf = NULL;
        return ESP_FAIL;
    }
    adc_data[0] = adc_data_buf->channel;
    memcpy(adc_data+1, adc_data_buf->data, adc_data_buf->len);
    free(adc_data_buf);
    adc_data_buf=NULL;
    return ESP_OK;
}
