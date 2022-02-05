#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "esp_attr.h"

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
#include "debug_cf.h"
#include "STM32_interconnect.h"
#include "config.h"
#define GPIO_HANDSHAKE 2

/* Private functions */
static void spiTask(void *arg);
volatile bool SPI_SLAVE_RX_CLPT = true;
bool NextSpiToRF = false, previousSpiToRF = false;
uint64_t now, last;
static xSemaphoreHandle spiDataReady;

STATIC_MEM_TASK_ALLOC(spiTask, SYSTEM_TASK_STACKSIZE);

static inline void Raise_STM32_SPI_ALT_Pin()
{
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << STM32_SPI_ALT));
}

static inline void Lower_STM32_SPI_ALT_Pin()
{
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << STM32_SPI_ALT));
}

// Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
IRAM_ATTR void slave_post_setup_cb(spi_slave_transaction_t *trans)
{
    Raise_STM32_SPI_ALT_Pin();
}

// Called after transaction is sent/received.
IRAM_ATTR void slave_post_trans_cb(spi_slave_transaction_t *trans)
{
    SPI_SLAVE_RX_CLPT = true;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    Lower_STM32_SPI_ALT_Pin();
    xSemaphoreGiveFromISR(spiDataReady, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

spi_slave_transaction_t SetupTransactionWithSTM32();
spi_slave_transaction_t SetupTransactionWithRFM69();
void ProcessTransactionFromSTM32(uint8_t *outputBuffer);
void ProcessTransactionFromRFM69(uint8_t *outputBuffer);

uint32_t i = 256, nextSize = 5, defaultSize = 5, maxSize = 128;
uint32_t badTransactions = 0;
SPI_ESP_PACKET_HEADER tx;
uint8_t *dout;
uint8_t *dat;

spi_bus_config_t buscfg = {
    .miso_io_num = STM32_SPI_MISO,
    .mosi_io_num = STM32_SPI_MOSI,
    .sclk_io_num = STM32_SPI_SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
    .flags = 0,
    .intr_flags = ESP_INTR_FLAG_IRAM};
// COM13909_SS

// Configuration for the SPI slave interface
spi_slave_interface_config_t slvcfg = {
    .mode = 1,
    .spics_io_num = STM32_SPI_SS,
    .queue_size = 7,
    .flags = 0,
    .post_setup_cb = slave_post_setup_cb,
    .post_trans_cb = slave_post_trans_cb};

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
    spiDataReady = xSemaphoreCreateBinary();

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
    ESP_ERROR_CHECK(ret);

    dout = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * maxSize, MALLOC_CAP_DMA);
    dat = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * maxSize, MALLOC_CAP_DMA);
    STATIC_MEM_TASK_CREATE(spiTask, spiTask, SPI_MAIN_TASK_NAME, NULL, SPI_MAIN_TASK_PRI);
}

void spiTask(void *arg)
{
    memset(dat, 0, 5);
    memset(dout, 0, maxSize);

    spi_slave_transaction_t PreviousSlaveTransaction = Slave_Transaction(dat, dout, 35);
    vTaskDelay(200);
    spi_slave_transaction_t *Transaction = &PreviousSlaveTransaction; // these need to be on the stack inside this tasks memory
    now = last = esp_timer_get_time();
    uint8_t *outputBuffer = malloc(sizeof(uint8_t) * maxSize);
    memset(outputBuffer, 0, maxSize);
    while (true)
    {
        i++;
        // xSemaphoreGive(SPI_READY);
        // xSemaphoreTake(SPI_READY, portMAX_DELAY);
        if (pdTRUE == xSemaphoreTake(spiDataReady, portMAX_DELAY))
        {
            SPI_SLAVE_RX_CLPT = false;
            spi_slave_get_trans_result(BUSS_1_HOST, &Transaction, 1);
            memcpy(outputBuffer, dout, maxSize);//Transaction->length);
            bool previousSpiToRFM69 = previousSpiToRF;
            if (NextSpiToRF)
            {
                NextSpiToRF = false;
                PreviousSlaveTransaction = SetupTransactionWithRFM69();
            }
            else
            {
                PreviousSlaveTransaction = SetupTransactionWithSTM32();
            }
            if (previousSpiToRFM69)
            {
                ProcessTransactionFromRFM69(outputBuffer);
            }
            else
            {
                ProcessTransactionFromSTM32(outputBuffer);
            }

            //  fflush(stdout);
        }
        // taskYIELD();
        // vTaskDelay(1);
        // portYIELD();
    }
}

spi_slave_transaction_t SetupTransactionWithRFM69()
{
    memset(dat, 0, 5);
    memset(dout, 0, 120);
    dat[0] = 0xaa;
    dat[1] = 0;
    dat[2] = 0xf0;
    dat[3] = 0x0f;
    dat[4] = 0;
    dat[5] = 0xaa;

    spi_slave_transaction_t PreviousTransaction = Slave_Transaction(dat, dout, 12 + 4);
    previousSpiToRF = true;
    return PreviousTransaction;
}

void ProcessTransactionFromRFM69(uint8_t *outputBuffer)
{
    now = esp_timer_get_time();
    DEBUG_PRINTI("%d   00=00 || %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d        %f n", badTransactions, outputBuffer[0], outputBuffer[1], outputBuffer[2], outputBuffer[3], outputBuffer[4], outputBuffer[5], outputBuffer[6], outputBuffer[7], outputBuffer[8], outputBuffer[9], outputBuffer[10], outputBuffer[11], outputBuffer[12], outputBuffer[13], outputBuffer[14], outputBuffer[15], outputBuffer[16], ((now - last) / 1000.0));
    last = esp_timer_get_time();
}

spi_slave_transaction_t SetupTransactionWithSTM32()
{
    // doing this before setting the contents of dat and dout assumes that the next transaction will not come for a while after this is setup
    // spi_slave_transaction_t PreviousTransaction = Slave_Transaction(dat, dout, nextSize + 4);

    memset(dat, 0, 5);
    memset(&tx, 0, sizeof(SPI_ESP_PACKET_HEADER));
    memset(dout, 0, 120);

    tx.nextSpiSize = nextSize;
    NextSpiToRF = true;
    tx.radioSendData = NextSpiToRF;
    if (NextSpiToRF)
    {
        tx.altCmd = 12;
    }
    else
    {
        tx.altCmd = 0xaa;
    }
    tx.motorSpeeed = 1;

    memcpy(dat, &tx, sizeof(SPI_ESP_PACKET_HEADER));
    dat[0] = calculate_cksum(dat + 1, 4 - 1);

    spi_slave_transaction_t PreviousTransaction = Slave_Transaction(dat, dout, nextSize + 4);
    previousSpiToRF = false;
    return PreviousTransaction;
}

void ProcessTransactionFromSTM32(uint8_t *outputBuffer)
{
    uint8_t crcIn = outputBuffer[0];
    uint8_t crcOut = calculate_cksum(outputBuffer + 1, 4 - 1);

    if (crcOut == crcIn && outputBuffer[1] != 0)
    {
        nextSize = outputBuffer[1];
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
    DEBUG_PRINTI("%d   %d=%d ||  %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d        %f n", badTransactions, crcIn, crcOut, outputBuffer[0], outputBuffer[1], outputBuffer[2], outputBuffer[3], outputBuffer[4], outputBuffer[5], outputBuffer[6], outputBuffer[7], outputBuffer[8], outputBuffer[9], outputBuffer[10], outputBuffer[11], outputBuffer[12], outputBuffer[13], outputBuffer[14], outputBuffer[15], outputBuffer[16], ((now - last) / 1000.0));
    last = esp_timer_get_time();
}