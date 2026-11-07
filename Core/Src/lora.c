#include "lora.h"

/* =========================================================
 * SPI HANDLE
 * ========================================================= */

extern SPI_HandleTypeDef hspi1;

/* =========================================================
 * INTERNAL VARIABLES
 * ========================================================= */

static LoRa_Error_t lastError =
    LORA_ERROR_NONE;

static uint32_t crcErrorCount = 0;

static uint32_t rxDoneCount = 0;

/* =========================================================
 * CHIP SELECT
 * ========================================================= */

static void LoRa_CS_Low(void)
{
    HAL_GPIO_WritePin(
        LORA_NSS_PORT,
        LORA_NSS_PIN,
        GPIO_PIN_RESET
    );
}

static void LoRa_CS_High(void)
{
    HAL_GPIO_WritePin(
        LORA_NSS_PORT,
        LORA_NSS_PIN,
        GPIO_PIN_SET
    );
}

/* =========================================================
 * WRITE REGISTER
 *
 * SX1278:
 *
 * Bit 7 = 1 -> WRITE
 * Bits 6:0 = register address
 * ========================================================= */

void LoRa_WriteRegister(uint8_t address,
                        uint8_t value)
{
    uint8_t data[2];

    data[0] = address | 0x80;
    data[1] = value;

    LoRa_CS_Low();

    HAL_SPI_Transmit(
        &hspi1,
        data,
        2,
        HAL_MAX_DELAY
    );

    LoRa_CS_High();
}

/* =========================================================
 * READ REGISTER
 *
 * FIX:
 * Use SPI full-duplex transfer.
 * ========================================================= */

uint8_t LoRa_ReadRegister(uint8_t address)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = address & 0x7F;
    tx[1] = 0x00;

    rx[0] = 0;
    rx[1] = 0;

    LoRa_CS_Low();

    HAL_SPI_TransmitReceive(
        &hspi1,
        tx,
        rx,
        2,
        HAL_MAX_DELAY
    );

    LoRa_CS_High();

    return rx[1];
}

/* =========================================================
 * RESET SX1278
 * ========================================================= */

static void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(
        LORA_RST_PORT,
        LORA_RST_PIN,
        GPIO_PIN_RESET
    );

    HAL_Delay(10);

    HAL_GPIO_WritePin(
        LORA_RST_PORT,
        LORA_RST_PIN,
        GPIO_PIN_SET
    );

    HAL_Delay(10);
}

/* =========================================================
 * SET OPERATING MODE
 * ========================================================= */

static void LoRa_SetMode(uint8_t mode)
{
    LoRa_WriteRegister(
        REG_OP_MODE,
        MODE_LONG_RANGE_MODE | mode
    );

    HAL_Delay(2);
}

/* =========================================================
 * SET FREQUENCY
 *
 * FRF = Frequency / (32MHz / 2^19)
 * ========================================================= */

static void LoRa_SetFrequency(uint32_t frequency)
{
    uint64_t frf;

    frf =
        ((uint64_t)frequency << 19)
        / 32000000ULL;

    LoRa_WriteRegister(
        REG_FRF_MSB,
        (uint8_t)(frf >> 16)
    );

    LoRa_WriteRegister(
        REG_FRF_MID,
        (uint8_t)(frf >> 8)
    );

    LoRa_WriteRegister(
        REG_FRF_LSB,
        (uint8_t)frf
    );
}

/* =========================================================
 * GET FREQUENCY
 * ========================================================= */

long LoRa_GetFrequency(void)
{
    uint32_t frf;

    frf =
        ((uint32_t)
         LoRa_ReadRegister(REG_FRF_MSB))
        << 16;

    frf |=
        ((uint32_t)
         LoRa_ReadRegister(REG_FRF_MID))
        << 8;

    frf |=
        LoRa_ReadRegister(REG_FRF_LSB);

    return (long)
        (((uint64_t)frf * 32000000ULL)
         >> 19);
}

/* =========================================================
 * SET BANDWIDTH
 *
 * BW = 7 -> 125 kHz
 * ========================================================= */

static void LoRa_SetBandwidth(uint8_t bw)
{
    uint8_t config;

    config =
        LoRa_ReadRegister(
            REG_MODEM_CONFIG_1
        );

    config &= 0x0F;

    config |=
        (uint8_t)(bw << 4);

    LoRa_WriteRegister(
        REG_MODEM_CONFIG_1,
        config
    );
}

/* =========================================================
 * GET BANDWIDTH
 * ========================================================= */

long LoRa_GetSignalBandwidth(void)
{
    uint8_t bw;

    bw =
        LoRa_ReadRegister(
            REG_MODEM_CONFIG_1
        ) >> 4;

    switch (bw)
    {
        case 0:
            return 7800;

        case 1:
            return 10400;

        case 2:
            return 15600;

        case 3:
            return 20800;

        case 4:
            return 31250;

        case 5:
            return 41700;

        case 6:
            return 62500;

        case 7:
            return 125000;

        case 8:
            return 250000;

        case 9:
            return 500000;

        default:
            return 0;
    }
}

/* =========================================================
 * SET SPREADING FACTOR
 * ========================================================= */

static void LoRa_SetSpreadingFactor(uint8_t sf)
{
    uint8_t config;

    if (sf < 6)
        sf = 6;

    if (sf > 12)
        sf = 12;

    config =
        LoRa_ReadRegister(
            REG_MODEM_CONFIG_2
        );

    config &= 0x0F;

    config |=
        (uint8_t)(sf << 4);

    LoRa_WriteRegister(
        REG_MODEM_CONFIG_2,
        config
    );

    /* -----------------------------------------------------
     * Special configuration for SF6.
     * ----------------------------------------------------- */

    if (sf == 6)
    {
        LoRa_WriteRegister(
            REG_DETECTION_OPTIMIZE,
            0xC5
        );

        LoRa_WriteRegister(
            REG_DETECTION_THRESHOLD,
            0x0C
        );
    }
    else
    {
        LoRa_WriteRegister(
            REG_DETECTION_OPTIMIZE,
            0xC3
        );

        LoRa_WriteRegister(
            REG_DETECTION_THRESHOLD,
            0x0A
        );
    }
}

/* =========================================================
 * GET SPREADING FACTOR
 * ========================================================= */

int LoRa_GetSpreadingFactor(void)
{
    return
        LoRa_ReadRegister(
            REG_MODEM_CONFIG_2
        ) >> 4;
}

/* =========================================================
 * SET CODING RATE
 *
 * denominator = 5 -> 4/5
 * denominator = 6 -> 4/6
 * denominator = 7 -> 4/7
 * denominator = 8 -> 4/8
 * ========================================================= */

static void LoRa_SetCodingRate4(uint8_t denominator)
{
    uint8_t config;

    if (denominator < 5)
        denominator = 5;

    if (denominator > 8)
        denominator = 8;

    config =
        LoRa_ReadRegister(
            REG_MODEM_CONFIG_1
        );

    config &= 0xF1;

    config |=
        (uint8_t)((denominator - 4) << 1);

    LoRa_WriteRegister(
        REG_MODEM_CONFIG_1,
        config
    );
}

/* =========================================================
 * GET CODING RATE
 * ========================================================= */

int LoRa_GetCodingRate4(void)
{
    uint8_t cr;

    cr =
        (LoRa_ReadRegister(
            REG_MODEM_CONFIG_1
        ) >> 1) & 0x07;

    return cr + 4;
}

/* =========================================================
 * DISABLE CRC
 * ========================================================= */

static void LoRa_DisableCrc(void)
{
    uint8_t config;

    config =
        LoRa_ReadRegister(
            REG_MODEM_CONFIG_2
        );

    config &= (uint8_t)~0x04;

    LoRa_WriteRegister(
        REG_MODEM_CONFIG_2,
        config
    );
}

/* =========================================================
 * CHECK CRC
 * ========================================================= */

bool LoRa_IsCrcEnabled(void)
{
    return
        (LoRa_ReadRegister(
            REG_MODEM_CONFIG_2
        ) & 0x04) != 0;
}

/* =========================================================
 * INITIALIZE SX1278
 * ========================================================= */

bool LoRa_Init(void)
{
    uint8_t version;

    lastError =
        LORA_ERROR_NONE;

    /* -----------------------------------------------------
     * NSS idle HIGH
     * ----------------------------------------------------- */

    LoRa_CS_High();

    HAL_Delay(10);

    /* -----------------------------------------------------
     * Hardware reset
     * ----------------------------------------------------- */

    LoRa_Reset();

    /* -----------------------------------------------------
     * Read chip version
     * ----------------------------------------------------- */

    version =
        LoRa_ReadRegister(
            REG_VERSION
        );

    if (version == 0x00 ||
        version == 0xFF)
    {
        lastError =
            LORA_ERROR_NO_SPI_RESPONSE;

        return false;
    }

    if (version != 0x12)
    {
        lastError =
            LORA_ERROR_WRONG_CHIP_ID;

        return false;
    }

    /* -----------------------------------------------------
     * Sleep mode
     * ----------------------------------------------------- */

    LoRa_SetMode(
        MODE_SLEEP
    );

    HAL_Delay(5);

    /* -----------------------------------------------------
     * Frequency = 433 MHz
     * ----------------------------------------------------- */

    LoRa_SetFrequency(
        433000000UL
    );

    /* -----------------------------------------------------
     * FIFO configuration
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_FIFO_TX_BASE_ADDR,
        0x00
    );

    LoRa_WriteRegister(
        REG_FIFO_RX_BASE_ADDR,
        0x00
    );

    /* -----------------------------------------------------
     * LNA
     *
     * LNA boost enabled.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_LNA,
        0x23
    );

    /* -----------------------------------------------------
     * Bandwidth = 125 kHz
     * ----------------------------------------------------- */

    LoRa_SetBandwidth(7);

    /* -----------------------------------------------------
     * Coding Rate = 4/5
     * ----------------------------------------------------- */

    LoRa_SetCodingRate4(5);

    /* -----------------------------------------------------
     * Spreading Factor = SF7
     * ----------------------------------------------------- */

    LoRa_SetSpreadingFactor(7);

    /* -----------------------------------------------------
     * CRC OFF
     * ----------------------------------------------------- */

    LoRa_DisableCrc();

    /* -----------------------------------------------------
     * Preamble = 8 symbols
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_PREAMBLE_MSB,
        0x00
    );

    LoRa_WriteRegister(
        REG_PREAMBLE_LSB,
        0x08
    );

    /* -----------------------------------------------------
     * Sync Word = 0x12
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_SYNC_WORD,
        0x12
    );

    /* -----------------------------------------------------
     * DIO0 mapping
     *
     * DIO0:
     * 00 = RxDone / TxDone
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_DIO_MAPPING_1,
        0x00
    );

    /* -----------------------------------------------------
     * TX POWER
     *
     * PA_BOOST selected.
     *
     * Output approximately +17 dBm.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_PA_CONFIG,
        0x8F
    );

    /* -----------------------------------------------------
     * OCP
     *
     * Default ~100 mA.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_OCP,
        0x20
    );

    /* -----------------------------------------------------
     * ModemConfig3
     *
     * AGC automatic.
     * Low data rate optimization OFF for SF7.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_MODEM_CONFIG_3,
        0x04
    );

    /* -----------------------------------------------------
     * Clear all pending IRQ flags.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_IRQ_FLAGS,
        0xFF
    );

    /* -----------------------------------------------------
     * No IRQ masking.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_IRQ_FLAGS_MASK,
        0x00
    );

    /* -----------------------------------------------------
     * Standby.
     * ----------------------------------------------------- */

    LoRa_SetMode(
        MODE_STDBY
    );

    /* -----------------------------------------------------
     * Verify operating mode.
     * ----------------------------------------------------- */

    if ((LoRa_GetOpMode() & 0x07)
        != MODE_STDBY)
    {
        lastError =
            LORA_ERROR_OPMODE_STUCK;

        return false;
    }

    /* -----------------------------------------------------
     * Verify sync word.
     * ----------------------------------------------------- */

    if (LoRa_ReadRegister(
            REG_SYNC_WORD
        ) != 0x12)
    {
        lastError =
            LORA_ERROR_WRITE_VERIFY;

        return false;
    }

    /* -----------------------------------------------------
     * Verify frequency.
     * ----------------------------------------------------- */

    {
        long frequency =
            LoRa_GetFrequency();

        if (frequency < 432900000L ||
            frequency > 433100000L)
        {
            lastError =
                LORA_ERROR_WRITE_VERIFY;

            return false;
        }
    }

    return true;
}

/* =========================================================
 * START CONTINUOUS RECEIVE
 * ========================================================= */

void LoRa_Receive(void)
{
    LoRa_WriteRegister(
        REG_IRQ_FLAGS,
        0xFF
    );

    LoRa_WriteRegister(
        REG_FIFO_ADDR_PTR,
        LoRa_ReadRegister(
            REG_FIFO_RX_BASE_ADDR
        )
    );

    LoRa_SetMode(
        MODE_RX_CONTINUOUS
    );
}

/* =========================================================
 * CHECK FOR PACKET
 * ========================================================= */

int LoRa_ParsePacket(void)
{
    uint8_t irq;

    irq =
        LoRa_ReadRegister(
            REG_IRQ_FLAGS
        );

    if (irq &
        IRQ_PAYLOAD_CRC_ERROR_MASK)
    {
        crcErrorCount++;

        LoRa_WriteRegister(
            REG_IRQ_FLAGS,
            IRQ_PAYLOAD_CRC_ERROR_MASK
        );
    }

    if (irq &
        IRQ_RX_DONE_MASK)
    {
        rxDoneCount++;

        return 1;
    }

    return 0;
}

/* =========================================================
 * READ PACKET FROM FIFO
 * ========================================================= */

uint8_t LoRa_Read(uint8_t *buffer,
                  uint8_t size)
{
    uint8_t length;
    uint8_t currentAddress;

    length =
        LoRa_ReadRegister(
            REG_RX_NB_BYTES
        );

    if (length > size)
        length = size;

    currentAddress =
        LoRa_ReadRegister(
            REG_FIFO_RX_CURRENT_ADDR
        );

    LoRa_WriteRegister(
        REG_FIFO_ADDR_PTR,
        currentAddress
    );

    /* -----------------------------------------------------
     * Read FIFO using full-duplex SPI.
     * ----------------------------------------------------- */

    if (length > 0)
    {
        uint8_t address = REG_FIFO & 0x7F;

        LoRa_CS_Low();

        HAL_SPI_Transmit(
            &hspi1,
            &address,
            1,
            HAL_MAX_DELAY
        );

        for (uint8_t i = 0;
             i < length;
             i++)
        {
            uint8_t tx = 0x00;
            uint8_t rx = 0x00;

            HAL_SPI_TransmitReceive(
                &hspi1,
                &tx,
                &rx,
                1,
                HAL_MAX_DELAY
            );

            buffer[i] = rx;
        }

        LoRa_CS_High();
    }

    /* -----------------------------------------------------
     * Clear RxDone.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_IRQ_FLAGS,
        IRQ_RX_DONE_MASK
    );

    return length;
}

/* =========================================================
 * PACKET RSSI
 *
 * 433 MHz:
 * RSSI = -164 + RegPktRssiValue
 * ========================================================= */

int16_t LoRa_PacketRssi(void)
{
    return (int16_t)(
        -164 +
        LoRa_ReadRegister(
            REG_PKT_RSSI_VALUE
        )
    );
}

/* =========================================================
 * PACKET SNR
 * ========================================================= */

float LoRa_PacketSnr(void)
{
    int8_t raw;

    raw =
        (int8_t)
        LoRa_ReadRegister(
            REG_PKT_SNR_VALUE
        );

    return raw / 4.0f;
}

/* =========================================================
 * GET IRQ FLAGS
 * ========================================================= */

uint8_t LoRa_GetIrqFlags(void)
{
    return
        LoRa_ReadRegister(
            REG_IRQ_FLAGS
        );
}

/* =========================================================
 * GET OPERATING MODE
 * ========================================================= */

uint8_t LoRa_GetOpMode(void)
{
    return
        LoRa_ReadRegister(
            REG_OP_MODE
        );
}

/* =========================================================
 * DIO0 STATE
 * ========================================================= */

bool LoRa_IsDio0High(void)
{
    return
        HAL_GPIO_ReadPin(
            LORA_DIO0_PORT,
            LORA_DIO0_PIN
        ) == GPIO_PIN_SET;
}

/* =========================================================
 * ERROR
 * ========================================================= */

LoRa_Error_t LoRa_GetLastError(void)
{
    return lastError;
}

/* =========================================================
 * ERROR STRING
 * ========================================================= */

const char *LoRa_GetLastErrorString(void)
{
    switch (lastError)
    {
        case LORA_ERROR_NONE:
            return "No error";

        case LORA_ERROR_NO_SPI_RESPONSE:
            return "No SPI response";

        case LORA_ERROR_WRONG_CHIP_ID:
            return "Wrong chip ID";

        case LORA_ERROR_WRITE_VERIFY:
            return "Register write verify failed";

        case LORA_ERROR_OPMODE_STUCK:
            return "Operating mode stuck";

        case LORA_ERROR_TX_TIMEOUT:
            return "TX timeout";

        default:
            return "Unknown error";
    }
}

/* =========================================================
 * COUNTERS
 * ========================================================= */

uint32_t LoRa_GetCrcErrorCount(void)
{
    return crcErrorCount;
}

uint32_t LoRa_GetRxDoneCount(void)
{
    return rxDoneCount;
}

/* =========================================================
 * BEGIN PACKET
 * ========================================================= */

void LoRa_BeginPacket(void)
{
    /* -----------------------------------------------------
     * Standby.
     * ----------------------------------------------------- */

    LoRa_SetMode(
        MODE_STDBY
    );

    /* -----------------------------------------------------
     * Clear IRQ flags.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_IRQ_FLAGS,
        0xFF
    );

    /* -----------------------------------------------------
     * Reset FIFO pointer.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_FIFO_ADDR_PTR,
        LoRa_ReadRegister(
            REG_FIFO_TX_BASE_ADDR
        )
    );

    /* -----------------------------------------------------
     * Reset payload length.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_PAYLOAD_LENGTH,
        0
    );
}

/* =========================================================
 * WRITE DATA TO FIFO
 * ========================================================= */

void LoRa_WriteBuffer(uint8_t *buffer,
                      uint8_t size)
{
    uint8_t address;

    address =
        REG_FIFO | 0x80;

    /* -----------------------------------------------------
     * Write payload to FIFO.
     * ----------------------------------------------------- */

    LoRa_CS_Low();

    HAL_SPI_Transmit(
        &hspi1,
        &address,
        1,
        HAL_MAX_DELAY
    );

    HAL_SPI_Transmit(
        &hspi1,
        buffer,
        size,
        HAL_MAX_DELAY
    );

    LoRa_CS_High();

    /* -----------------------------------------------------
     * Set payload length.
     * ----------------------------------------------------- */

    LoRa_WriteRegister(
        REG_PAYLOAD_LENGTH,
        size
    );
}

/* =========================================================
 * END PACKET / TRANSMIT
 * ========================================================= */

bool LoRa_EndPacket(void)
{
    uint32_t startTime;

    /* -----------------------------------------------------
     * Start TX.
     * ----------------------------------------------------- */

    LoRa_SetMode(
        MODE_TX
    );

    startTime =
        HAL_GetTick();

    /* -----------------------------------------------------
     * Wait for TxDone.
     * ----------------------------------------------------- */

    while (1)
    {
        uint8_t irq;

        irq =
            LoRa_ReadRegister(
                REG_IRQ_FLAGS
            );

        if (irq &
            IRQ_TX_DONE_MASK)
        {
            /* ---------------------------------------------
             * Clear TxDone.
             * --------------------------------------------- */

            LoRa_WriteRegister(
                REG_IRQ_FLAGS,
                IRQ_TX_DONE_MASK
            );

            /* ---------------------------------------------
             * Return to standby.
             * --------------------------------------------- */

            LoRa_SetMode(
                MODE_STDBY
            );

            return true;
        }

        /* -------------------------------------------------
         * 5 second safety timeout.
         * ------------------------------------------------- */

        if ((HAL_GetTick() - startTime)
            > 5000)
        {
            LoRa_SetMode(
                MODE_STDBY
            );

            lastError =
                LORA_ERROR_TX_TIMEOUT;

            return false;
        }
    }
}
