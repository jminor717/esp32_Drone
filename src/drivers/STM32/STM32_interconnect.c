#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#include "log.h"
#include "mem.h"
#include "queuemonitor.h"
#include "static_mem.h"
#include "cfassert.h"

#include "config/pin_config.h"
#include <../Common/Data_type.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define DEBUG_MODULE "SPI"

#include "STM32_interconnect.h"
#include "config.h"
#define GPIO_HANDSHAKE 2

/* Private functions */
static void spiTask(void *arg);
volatile bool SPI_SLAVE_RX_CLPT = true;

static xSemaphoreHandle SPI_READY;
STATIC_MEM_TASK_ALLOC(spiTask, SYSTEM_TASK_STACKSIZE);

// Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void slave_post_setup_cb(spi_slave_transaction_t *trans)
{
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << STM32_SPI_ALT));
}

// Called after transaction is sent/received.
void slave_post_trans_cb(spi_slave_transaction_t *trans)
{
    SPI_SLAVE_RX_CLPT = true;
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << STM32_SPI_ALT));

    //portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    //xSemaphoreGiveFromISR(SPI_READY, &xHigherPriorityTaskWoken);
    //xSemaphoreGive(SPI_READY);
    // if (xHigherPriorityTaskWoken)
    // {
    //     portYIELD_FROM_ISR();
    // }
}

uint32_t i = 256, nextSize = 5, defaultSize = 5, maxSize = 128;
uint32_t badTransactions = 0;
SPI_ESP_PACKET_HEADER tx;
uint8_t *dout;
uint8_t *dat;

spi_device_handle_t spi;
spi_bus_config_t buscfg = {
    .miso_io_num = STM32_SPI_MISO,
    .mosi_io_num = STM32_SPI_MOSI,
    .sclk_io_num = STM32_SPI_SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096};
spi_device_interface_config_t devcfg = {
#ifdef CONFIG_LCD_OVERCLOCK
    .clock_speed_hz = 26 * 1000 * 1000, // Clock out at 26 MHz
#else
    .clock_speed_hz = 125 * 1000, // Clock out at 1 MHz
#endif
    .mode = 0,          // SPI mode 0
    .spics_io_num = -1, // CS pin
    .queue_size = 7,    // We want to be able to queue 7 transactions at a time
    .address_bits = 0,  // 16,
    .dummy_bits = 0     // 8
    //.pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
};

// Configuration for the SPI slave interface
spi_slave_interface_config_t slvcfg = {
    .mode = 0,
    .spics_io_num = STM32_SPI_SS,
    .queue_size = 7,
    .flags = 0,
    .post_setup_cb = slave_post_setup_cb,
    .post_trans_cb = slave_post_trans_cb};

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void spi_sent_data(spi_device_handle_t spi, const uint8_t *data, uint8_t *out_data, int len)
{
    if (len == 0)
        return; // no need to send anything
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = len * 8;       // Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;       // Data
    t.rx_buffer = out_data;
    //  t.user = (void *)1;                         // D/C needs to be set to 1
    //  t.addr = 0b1001101001010101;
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

void Master_Transaction(spi_device_handle_t spi, uint8_t *dat, uint8_t *dout, uint32_t nextSize)
{
    gpio_set_level(STM32_SPI_SS, 0);
    ets_delay_us(20);
    spi_sent_data(spi, dat, dout, nextSize);
    ets_delay_us(80);
    gpio_set_level(STM32_SPI_SS, 1);
}

spi_slave_transaction_t Slave_Transaction(uint8_t *dat, uint8_t *dout, uint32_t nextSize)
{
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = nextSize * 8;
    t.tx_buffer = dat;
    t.rx_buffer = dout;
    /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
    initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
    by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
    .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
    data.
    */
    // spi_slave_transmit(BUSS_1_HOST, &t, portMAX_DELAY);
    spi_slave_queue_trans(BUSS_1_HOST, &t, portMAX_DELAY);
    return t;
}

/* Public functions */
void SPI_INIT(void)
{

    // Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << STM32_SPI_ALT)};

    // Configure handshake line as output
    gpio_config(&io_conf);

    gpio_set_direction(STM32_SPI_MOSI, GPIO_MODE_INPUT);
    gpio_set_direction(STM32_SPI_SCK, GPIO_MODE_INPUT);
    gpio_set_direction(STM32_SPI_MISO, GPIO_MODE_OUTPUT);
    gpio_set_direction(STM32_SPI_SS, GPIO_MODE_INPUT);
    gpio_set_pull_mode(STM32_SPI_MISO, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(STM32_SPI_MOSI, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(STM32_SPI_SCK, GPIO_PULLDOWN_ONLY);

    // Initialize SPI slave interface
    esp_err_t ret = spi_slave_initialize(BUSS_1_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO); // SPI_DMA_DISABLED SPI_DMA_CH_AUTO
    assert(ret == ESP_OK);

    dout = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * maxSize, MALLOC_CAP_DMA);
    dat = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * maxSize, MALLOC_CAP_DMA);
    STATIC_MEM_TASK_CREATE(spiTask, spiTask, SPI_MAIN_TASK_NAME, NULL, SPI_MAIN_TASK_PRI);
}

void spiTask(void *arg)
{
    memset(dat, 0, 5);
    memset(dout, 0, maxSize);

    spi_slave_transaction_t PreviousSlaveTransaction = Slave_Transaction(dat, dout, nextSize);
    spi_slave_transaction_t *Transaction = &PreviousSlaveTransaction;
    uint64_t now = esp_timer_get_time(), last = esp_timer_get_time();
    while (true)
    {
        //xSemaphoreGive(SPI_READY);
        //xSemaphoreTake(SPI_READY, portMAX_DELAY);
        if (SPI_SLAVE_RX_CLPT)
        {
            SPI_SLAVE_RX_CLPT = false;
            spi_slave_get_trans_result(BUSS_1_HOST, &Transaction, 1);

            uint8_t crcIn = dout[0];
            uint8_t crcOut = calculate_cksum(dout + 1, 4 - 1);

            if (crcOut == crcIn && dout[1] != 0)
            {
                nextSize = dout[1];
            }
            else
            {
                nextSize = defaultSize;
            }
            if (crcOut != crcIn)
            {
                badTransactions++;
            }
            now = esp_timer_get_time();
            printf("%d   %d=%d || %d %d %d %d %d  ||  %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d        %f\n", badTransactions, crcIn, crcOut, dat[0], dat[1], dat[2], dat[3], dat[4], dout[0], dout[1], dout[2], dout[3], dout[4], dout[5], dout[6], dout[7], dout[8], dout[9], dout[10], dout[11], dout[12], dout[13], dout[14], dout[15], dout[16], ((now - last) / 1000.0));
            last = now;

            fflush(stdout);

            spi_slave_free(BUSS_1_HOST);

            spi_bus_add_device();

                esp_err_t ret = spi_slave_initialize(BUSS_1_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO); // SPI_DMA_DISABLED SPI_DMA_CH_AUTO
            assert(ret == ESP_OK);

            // dat = (uint8_t *)heap_caps_realloc(dat, sizeof(uint8_t) * nextSize, MALLOC_CAP_DMA);
            // dout = (uint8_t *)heap_caps_realloc(dout, sizeof(uint8_t) * nextSize, MALLOC_CAP_DMA);
            memset(dat, 0, 5);
            memset(&tx, 0, sizeof(SPI_ESP_PACKET_HEADER));
            memset(dout, 0, 120);

            tx.nextSpiSize = nextSize;
            tx.radioSendData = 0;
            tx.motorSpeeed = 1;
            tx.altCmd = 0xaa;
            memcpy(dat, &tx, sizeof(SPI_ESP_PACKET_HEADER));
            dat[0] = calculate_cksum(dat + 1, 4 - 1);

            // Master_Transaction(spi, dat, dout, nextSize);
            PreviousSlaveTransaction = Slave_Transaction(dat, dout, nextSize);
        }
        // taskYIELD();
         vTaskDelay(1);
    }
}