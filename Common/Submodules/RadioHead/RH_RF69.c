// RH_RF69.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF69.cpp,v 1.31 2019/09/02 05:21:52 mikem Exp $
#include "string.h"

#include "RH_RF69.h"
#include "config/pin_config.h"
#include "esp_intr_alloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
// It is important to keep the modulation index for FSK between 0.5 and 10
// modulation index = 2 * Fdev / BR
// Note that I have not had much success with FSK with Fd > ~5
// You have to construct these by hand, using the data from the RF69 Datasheet :-(
// or use the SX1231 starter kit software (Ctl-Alt-N to use that without a connected radio)
#define CONFIG_FSK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE)
#define CONFIG_GFSK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0)
#define CONFIG_OOK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_OOK | RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE)

// Choices for RH_RF69_REG_37_PACKETCONFIG1:
#define CONFIG_NOWHITE (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_NONE | RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE)
#define CONFIG_WHITE (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_WHITENING | RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE)
#define CONFIG_MANCHESTER (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_MANCHESTER | RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE)
static const ModemConfig MODEM_CONFIG_TABLE[] =
    {
        // PROGMEM
        //   02,        03,   04,   05,   06,   19,   1a,  37
        //  FSK, No Manchester, no shaping, whitening, CRC, no address filtering
        //  AFC BW == RX BW == 2 x bit rate
        //  Low modulation indexes of ~ 1 at slow speeds do not seem to work very well. Choose MI of 2.
        {CONFIG_FSK, 0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2Fd5
        {CONFIG_FSK, 0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2_4Fd4_8
        {CONFIG_FSK, 0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb4_8Fd9_6

        {CONFIG_FSK, 0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb9_6Fd19_2
        {CONFIG_FSK, 0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE}, // FSK_Rb19_2Fd38_4
        {CONFIG_FSK, 0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE}, // FSK_Rb38_4Fd76_8

        {CONFIG_FSK, 0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE}, // FSK_Rb57_6Fd120
        {CONFIG_FSK, 0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE}, // FSK_Rb125Fd125
        {CONFIG_FSK, 0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE}, // FSK_Rb250Fd250
        {CONFIG_FSK, 0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE}, // FSK_Rb55555Fd50

        //  02,        03,   04,   05,   06,   19,   1a,  37
        // GFSK (BT=1.0), No Manchester, whitening, CRC, no address filtering
        // AFC BW == RX BW == 2 x bit rate
        {CONFIG_GFSK, 0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf5, CONFIG_WHITE}, // GFSK_Rb2Fd5
        {CONFIG_GFSK, 0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb2_4Fd4_8
        {CONFIG_GFSK, 0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb4_8Fd9_6

        {CONFIG_GFSK, 0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb9_6Fd19_2
        {CONFIG_GFSK, 0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE}, // GFSK_Rb19_2Fd38_4
        {CONFIG_GFSK, 0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE}, // GFSK_Rb38_4Fd76_8

        {CONFIG_GFSK, 0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE}, // GFSK_Rb57_6Fd120
        {CONFIG_GFSK, 0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE}, // GFSK_Rb125Fd125
        {CONFIG_GFSK, 0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE}, // GFSK_Rb250Fd250
        {CONFIG_GFSK, 0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE}, // GFSK_Rb55555Fd50

        //  02,        03,   04,   05,   06,   19,   1a,  37
        // OOK, No Manchester, no shaping, whitening, CRC, no address filtering
        // with the help of the SX1231 configuration program
        // AFC BW == RX BW
        // All OOK configs have the default:
        // Threshold Type: Peak
        // Peak Threshold Step: 0.5dB
        // Peak threshiold dec: ONce per chip
        // Fixed threshold: 6dB
        {CONFIG_OOK, 0x7d, 0x00, 0x00, 0x10, 0x88, 0x88, CONFIG_WHITE}, // OOK_Rb1Bw1
        {CONFIG_OOK, 0x68, 0x2b, 0x00, 0x10, 0xf1, 0xf1, CONFIG_WHITE}, // OOK_Rb1_2Bw75
        {CONFIG_OOK, 0x34, 0x15, 0x00, 0x10, 0xf5, 0xf5, CONFIG_WHITE}, // OOK_Rb2_4Bw4_8
        {CONFIG_OOK, 0x1a, 0x0b, 0x00, 0x10, 0xf4, 0xf4, CONFIG_WHITE}, // OOK_Rb4_8Bw9_6
        {CONFIG_OOK, 0x0d, 0x05, 0x00, 0x10, 0xf3, 0xf3, CONFIG_WHITE}, // OOK_Rb9_6Bw19_2
        {CONFIG_OOK, 0x06, 0x83, 0x00, 0x10, 0xf2, 0xf2, CONFIG_WHITE}, // OOK_Rb19_2Bw38_4
        {CONFIG_OOK, 0x03, 0xe8, 0x00, 0x10, 0xe2, 0xe2, CONFIG_WHITE}, // OOK_Rb32Bw64

        //    { CONFIG_FSK,  0x68, 0x2b, 0x00, 0x52, 0x55, 0x55, CONFIG_WHITE}, // works: Rb1200 Fd 5000 bw10000, DCC 400
        //    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x52, 0x52, CONFIG_WHITE}, // works 10/40/80
        //    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x53, 0x53, CONFIG_WHITE}, // works 10/40/40

};

spi_bus_config_t buscfg = {
    .miso_io_num = STM32_SPI_MISO,
    .mosi_io_num = STM32_SPI_MOSI,
    .sclk_io_num = STM32_SPI_SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
    .flags = 0,
    .intr_flags = ESP_INTR_FLAG_IRAM};

spi_device_handle_t RFM69spi;
spi_device_interface_config_t RFM69cfg = {
    .clock_speed_hz = 125 * 1000, // Clock out at 1 MHz
    .mode = 1,                    // SPI mode 0
    .spics_io_num = -1,           // COM13909_SS,  // CS pin
    .queue_size = 7,              // We want to be able to queue 7 transactions at a time
    .address_bits = 0,            // 16,
    .dummy_bits = 0               // 8
    //.pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
};

void RH_RF69(uint8_t slaveSelectPin, uint8_t interruptPin, uint8_t spi) //, RHGenericSPI &spi
                                                                        // : RHSPIDriver(slaveSelectPin, spi)
{
    _slaveSelectPin = slaveSelectPin;
    _interruptPin = interruptPin;
    _idleMode = RH_RF69_OPMODE_MODE_STDBY;
    _myInterruptIndex = 0xff; // Not allocated yet
}

void setIdleMode(uint8_t idleMode)
{
    _idleMode = idleMode;
}

void Master_Transaction(spi_device_handle_t spi, uint8_t *dat, uint8_t *dout, uint32_t nextSize)
{
    gpio_set_level(_slaveSelectPin, 0);
    ets_delay_us(20);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = nextSize * 8;  // Len is in bytes, transaction length is in bits.
    t.tx_buffer = dat;        // Data
    t.rx_buffer = dout;
    //  t.user = (void *)1;                         // D/C needs to be set to 1
    //  t.addr = 0b1001101001010101;
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);
    // ets_delay_us(80);
    gpio_set_level(_slaveSelectPin, 1);
    // return t;
}

void spiWrite(uint8_t reg, uint8_t val)
{
    uint8_t dat[2] = {reg, val};
    uint8_t dout[2] = {0, 0}; // TODO: test to see if the SPI driver is ok with this being NULL
    Master_Transaction(RFM69spi, dat, dout, 2);
}

void spiBurstWrite(uint8_t reg, uint8_t *src, uint8_t len)
{
    uint8_t *dat = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * len + 1, MALLOC_CAP_DMA);
    dat[0] = reg;
    memcpy(dat + 1, src, len);
    // uint8_t dout[len + 1] = {0}; // TODO: test to see if the SPI driver is ok with this being NULL
    uint8_t *dout = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * len + 1, MALLOC_CAP_DMA);
    Master_Transaction(RFM69spi, src, dout, 2);
}

uint8_t spiRead(uint8_t reg)
{
    uint8_t dat[2] = {reg, 0};
    uint8_t dout[2] = {0, 0};
    Master_Transaction(RFM69spi, dat, dout, 2);
    return dout[1];
}

bool RH_RF69_init()
{
    // Determine the interrupt number that corresponds to the interruptPin
    //!  int interruptNumber = digitalPinToInterrupt(_interruptPin);

#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
#endif

    esp_err_t ret;
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI3_HOST, &RFM69cfg, &RFM69spi);
    ESP_ERROR_CHECK(ret);

    // Tell the low level SPI interface we will use SPI within this interrupt
    //!   spiUsingInterrupt(interruptNumber);

    // Get the device type and check it
    // This also tests whether we are really connected to a device
    // My test devices return 0x24
    _deviceType = spiRead(RH_RF69_REG_10_VERSION);
    if (_deviceType == 00 ||
        _deviceType == 0xff)
        return false;

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    // pinMode(_interruptPin, INPUT);

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // ON some devices, notably most Arduino's, the interrupt pin passed in is actuallt the
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knowledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
        // First run, no interrupt allocated yet
        if (_interruptCount <= RH_RF69_NUM_INTERRUPTS)
            _myInterruptIndex = _interruptCount++;
        else
            return false; // Too many devices, not enough interrupt vectors
    }

    //! attachInterrupt(interruptNumber, isr0, RISING);

    setModeIdle();

    // Configure important RH_RF69 registers
    // Here we set up the standard packet format for use by the RH_RF69 library:
    // 4 bytes preamble
    // 2 SYNC words 2d, d4
    // 2 CRC CCITT octets computed on the header, length and data (this in the modem config data)
    // 0 to 60 bytes data
    // RSSI Threshold -114dBm
    // We dont use the RH_RF69s address filtering: instead we prepend our own headers to the beginning
    // of the RH_RF69 payload
    spiWrite(RH_RF69_REG_3C_FIFOTHRESH, RH_RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY | 0x0f); // thresh 15 is default
    // RSSITHRESH is default
    //    spiWrite(RH_RF69_REG_29_RSSITHRESH, 220); // -110 dbM
    // SYNCCONFIG is default. SyncSize is set later by setSyncWords()
    //    spiWrite(RH_RF69_REG_2E_SYNCCONFIG, RH_RF69_SYNCCONFIG_SYNCON); // auto, tolerance 0
    // PAYLOADLENGTH is default
    //    spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, RH_RF69_FIFO_SIZE); // max size only for RX
    // PACKETCONFIG 2 is default
    spiWrite(RH_RF69_REG_6F_TESTDAGC, RH_RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF);
    // If high power boost set previously, disable it
    spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);

    // The following can be changed later by the user if necessary.
    // Set up default configuration
    uint8_t syncwords[] = {0x2d, 0xd4};
    setSyncWords(syncwords, sizeof(syncwords)); // Same as RF22's
    // Reasonably fast and reliable default speed and modulation
    setModemConfig(GFSK_Rb250Fd250);

    // 3 would be sufficient, but this is the same as RF22's
    setPreambleLength(4);
    // An innocuous ISM frequency, same as RF22's
    setFrequency(915.0);
    // No encryption
    setEncryptionKey(NULL);
    // +13dBm, same as power-on default RH_RF69_DEFAULT_HIGHPOWER
    setTxPower(20, true);

    return true;
}

// C++ level interrupt handler for this instance
// RH_RF69 is unusual in that it has several interrupt lines, and not a single, combined one.
// On Moteino, only one of the several interrupt lines (DI0) from the RH_RF69 is connnected to the processor.
// We use the single interrupt line to get PACKETSENT and PAYLOADREADY interrupts.
void handleInterrupt()
{
    // Get the interrupt cause
    uint8_t irqflags2 = spiRead(RH_RF69_REG_28_IRQFLAGS2);
    if (_mode == RHModeTx && (irqflags2 & RH_RF69_IRQFLAGS2_PACKETSENT))
    {
        // A transmitter message has been fully sent
        setModeIdle(); // Clears FIFO
        _txGood++;
        //	Serial.println("PACKETSENT");
    }

    // Must look for PAYLOADREADY, not CRCOK, since only PAYLOADREADY occurs _after_ AES decryption
    // has been done
    if (_mode == RHModeRx && (irqflags2 & RH_RF69_IRQFLAGS2_PAYLOADREADY))
    {
        // A complete message has been received with good CRC
        _lastRssi = -((int8_t)(spiRead(RH_RF69_REG_24_RSSIVALUE) >> 1));
        _lastPreambleTime = xTaskGetTickCount();

        setModeIdle();
        // Save it in our buffer
        readFifo();
        //	Serial.println("PAYLOADREADY");
    }
}

// Low level function reads the FIFO and checks the address
void readFifo()
{
    uint8_t *dat, *dout;
    // allocate arrays as RH_RF69_FIFO_SIZE + 1 to account for the RH_RF69_REG_00_FIFO byte
    dout = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * RH_RF69_FIFO_SIZE + 1, MALLOC_CAP_DMA);
    dat = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * RH_RF69_FIFO_SIZE + 1, MALLOC_CAP_DMA);
    memset(dat, 0, RH_RF69_FIFO_SIZE + 1);
    memset(dout, 0, RH_RF69_FIFO_SIZE + 1);
    dat[0] = RH_RF69_REG_00_FIFO;
    esp_err_t ret;
    spi_transaction_t tLen;
    spi_transaction_t tFifo;
    gpio_set_level(_slaveSelectPin, 0);

    memset(&tLen, 0, sizeof(tLen));
    memset(&tFifo, 0, sizeof(tFifo));
    tLen.tx_buffer = dat;
    tLen.rx_buffer = dout;
    tLen.length = 2 * 8;
    tFifo.tx_buffer = dat + 2; // just reuse the previously allocated memory
    tFifo.rx_buffer = dout + 2;
    ret = spi_device_polling_transmit(RFM69spi, &tLen); // read the first byte of the fifo to determine how long the received packet is
    assert(ret == ESP_OK);
    tFifo.length = dout[1] * 8;
    ret = spi_device_polling_transmit(RFM69spi, &tFifo); // read the rest of the fifo
    assert(ret == ESP_OK);
    // ets_delay_us(80);

    gpio_set_level(_slaveSelectPin, 1);
    //_rxGood++;
    _rxBufValid = true;
    // _rxHeaderFrom = dout[2];
    // _rxHeaderId = dout[3];
    // _rxHeaderFlags = dout[4];
    memcpy(_buf, dout + 2, dout[1] - 2);
    //  Any junk remaining in the FIFO will be cleared next time we go to receive mode.
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF69.
// 3 interrupts allows us to have 3 different devices
// RH_INTERRUPT_ATTR
void isr0()
{
    //! if (_deviceForInterrupt[0])
    //!     _deviceForInterrupt[0]->handleInterrupt();
}

int8_t temperatureRead()
{
    // Caution: must be ins standby.
    //    setModeIdle();
    spiWrite(RH_RF69_REG_4E_TEMP1, RH_RF69_TEMP1_TEMPMEASSTART); // Start the measurement
    while (spiRead(RH_RF69_REG_4E_TEMP1) & RH_RF69_TEMP1_TEMPMEASRUNNING)
        ;                                       // Wait for the measurement to complete
    return 166 - spiRead(RH_RF69_REG_4F_TEMP2); // Very approximate, based on observation
}

bool setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (uint32_t)((centre * 1000000.0) / RH_RF69_FSTEP);
    spiWrite(RH_RF69_REG_07_FRFMSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF69_REG_08_FRFMID, (frf >> 8) & 0xff);
    spiWrite(RH_RF69_REG_09_FRFLSB, frf & 0xff);
    return true;
}

int8_t rssiRead()
{
    // Force a new value to be measured
    // Hmmm, this hangs forever!
    //? i believe Rssi is only calculated after 2 bit periods of an incoming transmition that are part of the Preamble or Sync Word, investigate?
#if 0
    spiWrite(RH_RF69_REG_23_RSSICONFIG, RH_RF69_RSSICONFIG_RSSISTART);
    while (!(spiRead(RH_RF69_REG_23_RSSICONFIG) & RH_RF69_RSSICONFIG_RSSIDONE))
	;
#endif
    return -((int8_t)(spiRead(RH_RF69_REG_24_RSSIVALUE) >> 1));
}

void setOpMode(uint8_t mode, bool waitForModeReady)
{
    uint8_t opmode = spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (mode & RH_RF69_OPMODE_MODE);
    spiWrite(RH_RF69_REG_01_OPMODE, opmode);

    if (waitForModeReady)
    {
        // Wait for mode to change.
        while (!(spiRead(RH_RF69_REG_27_IRQFLAGS1) & RH_RF69_IRQFLAGS1_MODEREADY))
            ; // TODO: connect DIO5 and use a samafore to wait for the mode ready interupt here, or poll the line state
    }
}

void setModeIdle()
{
    if (_mode != RHModeIdle)
    {
        if (_paInHighPowerMode && _power >= 18)
        {
            _paInHighPowerMode = false;
            // If high power boost, return power amp to receive mode
            spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
            spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
        }
        setOpMode(_idleMode, true);
        _mode = RHModeIdle;
    }
}

bool RH_RF69_sleep()
{
    if (_mode != RHModeSleep)
    {
        spiWrite(RH_RF69_REG_01_OPMODE, RH_RF69_OPMODE_MODE_SLEEP);
        _mode = RHModeSleep;
    }
    return true;
}

void setModeRx()
{
    if (_mode != RHModeRx)
    {
        if (_paInHighPowerMode && _power >= 18)
        {
            _paInHighPowerMode = false;
            // If high power boost, return power amp to receive mode
            spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
            spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
        }
        spiWrite(RH_RF69_REG_25_DIOMAPPING1, RF69_DIO0_PayloadReady_TxReady); // Set interrupt line 0 PayloadReady
        setOpMode(RH_RF69_OPMODE_MODE_RX, true);                              // Clears FIFO
        _mode = RHModeRx;
    }
}

void setModeTx(bool waitForModeReady)
{
    if (_mode != RHModeTx)
    {
        if (!_paInHighPowerMode && _power >= 18)
        {
            _paInHighPowerMode = true;
            // Set high power boost mode
            // Note that OCP defaults to ON so no need to change that.
            spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_BOOST);
            spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_BOOST);
        }
        spiWrite(RH_RF69_REG_25_DIOMAPPING1, RF69_DIO0_CrcOk_PacketSent); // Set interrupt line 0 PacketSent
        setOpMode(RH_RF69_OPMODE_MODE_TX, waitForModeReady);              // Clears FIFO
        _mode = RHModeTx;
    }
}

void setTxPower(int8_t power, bool ishighpowermodule)
{
    _power = power;
    uint8_t palevel;

    if (ishighpowermodule)
    {
        if (_power < -2)
            _power = -2; // RFM69HW only works down to -2.
        if (_power <= 13)
        {
            // -2dBm to +13dBm
            // Need PA1 exclusivelly on RFM69HW
            palevel = RH_RF69_PALEVEL_PA1ON | ((_power + 18) &
                                               RH_RF69_PALEVEL_OUTPUTPOWER);
        }
        else if (_power >= 18)
        {
            // +18dBm to +20dBm
            // Need PA1+PA2
            // Also need PA boost settings change when tx is turned on and off, see setModeTx()
            palevel = RH_RF69_PALEVEL_PA1ON | RH_RF69_PALEVEL_PA2ON | ((_power + 11) & RH_RF69_PALEVEL_OUTPUTPOWER);
        }
        else
        {
            // +14dBm to +17dBm
            // Need PA1+PA2
            palevel = RH_RF69_PALEVEL_PA1ON | RH_RF69_PALEVEL_PA2ON | ((_power + 14) & RH_RF69_PALEVEL_OUTPUTPOWER);
        }
    }
    else
    {
        if (_power < -18)
            _power = -18;
        if (_power > 13)
            _power = 13; // limit for RFM69W
        palevel = RH_RF69_PALEVEL_PA0ON | ((_power + 18) & RH_RF69_PALEVEL_OUTPUTPOWER);
    }
    spiWrite(RH_RF69_REG_11_PALEVEL, palevel);
}

// Sets registers from a canned modem configuration structure
void setModemRegisters(const ModemConfig *config)
{
    spiBurstWrite(RH_RF69_REG_02_DATAMODUL, &config->reg_02, 5);
    spiBurstWrite(RH_RF69_REG_19_RXBW, &config->reg_19, 2);
    spiWrite(RH_RF69_REG_37_PACKETCONFIG1, config->reg_37);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

void setPreambleLength(uint16_t bytes)
{
    spiWrite(RH_RF69_REG_2C_PREAMBLEMSB, bytes >> 8);
    spiWrite(RH_RF69_REG_2D_PREAMBLELSB, bytes & 0xff);
}

void setSyncWords(const uint8_t *syncWords, uint8_t len)
{
    uint8_t syncconfig = spiRead(RH_RF69_REG_2E_SYNCCONFIG);
    if (syncWords && len && len <= 4)
    {
        spiBurstWrite(RH_RF69_REG_2F_SYNCVALUE1, syncWords, len);
        syncconfig |= RH_RF69_SYNCCONFIG_SYNCON;
    }
    else
        syncconfig &= ~RH_RF69_SYNCCONFIG_SYNCON;
    syncconfig &= ~RH_RF69_SYNCCONFIG_SYNCSIZE;
    syncconfig |= (len - 1) << 3;
    spiWrite(RH_RF69_REG_2E_SYNCCONFIG, syncconfig);
}

void setEncryptionKey(uint8_t *key)
{
    if (key)
    {
        spiBurstWrite(RH_RF69_REG_3E_AESKEY1, key, 16);
        spiWrite(RH_RF69_REG_3D_PACKETCONFIG2, spiRead(RH_RF69_REG_3D_PACKETCONFIG2) | RH_RF69_PACKETCONFIG2_AESON);
    }
    else
    {
        spiWrite(RH_RF69_REG_3D_PACKETCONFIG2, spiRead(RH_RF69_REG_3D_PACKETCONFIG2) & ~RH_RF69_PACKETCONFIG2_AESON);
    }
}
// TODO
bool available()
{
    if (_mode == RHModeTx)
        return false;
    setModeRx(); // Make sure we are receiving
    return _rxBufValid;
}
// TODO
bool recv(uint8_t *buf, uint8_t *len)
{
    if (!available())
        return false;

    if (buf && len)
    {
        // ATOMIC_BLOCK_START;
        if (*len > _bufLen)
            *len = _bufLen;
        memcpy(buf, _buf, *len);
        // ATOMIC_BLOCK_END;
    }
    _rxBufValid = false; // Got the most recent message
                         //    printBuffer("recv:", buf, *len);
    return true;
}

/** // TODO
 bool RHGenericDriver::waitPacketSent()
{
    while (_mode == RHModeTx)
    YIELD; // Wait for any previous transmit to finish
    return true;
}

// Wait until no channel activity detected or timeout
bool RHGenericDriver::waitCAD()
{
    if (!_cad_timeout)
    return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
    unsigned long t = millis();
    while (isChannelActive())
    {
         if (millis() - t > _cad_timeout)
         return false;
#if (RH_PLATFORM == RH_PLATFORM_STM32) // stdlib on STMF103 gets confused if random is redefined
     delay(_random(1, 10) * 100);
#else
         delay(random(1, 10) * 100); // Should these values be configurable? Macros?
#endif
    }

    return true;
}

*/

bool send(const uint8_t *data, uint8_t len)
{
    if (len > RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN)
        return false;

    // waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle(); // Prevent RX while filling the fifo

    // if (!waitCAD())
    //     return false; // Check channel activity

    uint8_t *dat, *dout;
    // allocate arrays as RH_RF69_FIFO_SIZE + 1 to account for the RH_RF69_REG_00_FIFO byte
    dout = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * RH_RF69_FIFO_SIZE + 1, MALLOC_CAP_DMA);
    dat = (uint8_t *)heap_caps_malloc(sizeof(uint8_t) * RH_RF69_FIFO_SIZE + 1, MALLOC_CAP_DMA);
    //  memset(dat, 0, RH_RF69_FIFO_SIZE + 1);
    memset(dout, 0, RH_RF69_FIFO_SIZE + 1);
    dat[0] = RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK;
    dat[1] = len;
    memcpy(dat + 1, data, len);

    // len += RH_RF69_HEADER_LEN;
    //  dat[1] = _txHeaderTo;
    //  dat[2] = _txHeaderFrom;
    //  dat[3] = _txHeaderId;
    //  dat[4] = _txHeaderFlags;

    esp_err_t ret;
    spi_transaction_t tFifo;
    gpio_set_level(_slaveSelectPin, 0);

    memset(&tFifo, 0, sizeof(tFifo));
    tFifo.tx_buffer = dat; // just reuse the previously allocated memory
    tFifo.rx_buffer = dout;
    tFifo.length = len * 8;

    setModeTx(false);                                    // Start the transmitter without waiting so that we start filing the fifo before the device is ready to transmit
    ret = spi_device_polling_transmit(RFM69spi, &tFifo); // read the rest of the fifo
    assert(ret == ESP_OK);
    // ets_delay_us(80);

    gpio_set_level(_slaveSelectPin, 1);

    return true;
}

uint8_t maxMessageLength()
{
    return RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN;
}

bool printRegister(uint8_t reg)
{
#ifdef RH_HAVE_SERIAL
    Serial.print(reg, HEX);
    Serial.print(" ");
    Serial.println(spiRead(reg), HEX);
#endif
    return true;
}

bool printRegisters()
{
    uint8_t i;
    for (i = 0; i < 0x50; i++)
        printRegister(i);
    // Non-contiguous registers
    printRegister(RH_RF69_REG_58_TESTLNA);
    printRegister(RH_RF69_REG_6F_TESTDAGC);
    printRegister(RH_RF69_REG_71_TESTAFC);

    return true;
}
