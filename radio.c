// -----------------------------------------------------------------------------
// File     : radio.c
// Product  : RF_V1
// Created  : 13-Feb-2018
// Author   : Gerard Zennipman
// Comment  : This file contains SI4455 radio transmit and receive functions.
// -----------------------------------------------------------------------------

#include <stdint.h>
#include "chip.h"
#include "radio.h"
#include "stringfunctions.h"
#include "spi.h"

// -----------------------------------------------------------------------------
// Constant definitions
// -----------------------------------------------------------------------------

#define RF_SHUTDOWN             (18)    // GPIO18 = LPC824_PIN31
#define RF_IRQN                 (17)    // GPIO17 = LPC824_PIN32
#define RF_GPIO1                (19)    // GPIO19 = LPC824_PIN30
#define RF_GPIO2                (20)    // GPIO20 = LPC824_PIN29

#define RF_SHUTDOWN_ACTIVE      (1)     // Pin ShutDown active 3V3
#define RF_SHUTDOWN_INACTIVE    (0)     // Pin ShutDown inactive 0V
#define RF_IRQN_ACTIVE          (0)     // Pin InterruptRequest active 0V
#define RF_IRQN_INACTIVE        (1)     // Pin InterruptRequest inactive 3V3
#define RF_GPIO1_CTS_ACTIVE     (1)     // Pin GPIO1_ClearToSend active 3V3
#define RF_GPIO1_CTS_INACTIVE   (0)     // Pin GPIO1_ClearToSend inactive 0V

#define RADIO_DELAY_20_USEC     (20)    // Delay 20 usec
#define RADIO_DELAY_1_SEC       (1000)  // Delay 1000 msec

#define EZ_CONFIG_ARRAY_VALID   (0x00)  // EZConfigArray is valid
#define EZ_CONFIG_ARRAY_VALID   (0x00)  // EZConfigArray is valid
#define PH_PACKET_SENT_PEND     (0x20)  // Pending interrupt packet transmitted
#define PH_PACKET_RX_PEND       (0x10)  // Pending interrupt packet valid received
#define PH_CRC_ERROR_PEND       (0x08)  // Pending interrupt packet invalid received

// -----------------------------------------------------------------------------
// Type definitions
// -----------------------------------------------------------------------------

typedef enum
{
    RADIO_SM_TRM_01_POWER_ON_DELAY_1_SEC = 1,
    RADIO_SM_TRM_02_POWER_UP_SHUTDOWN_20_USEC,
    RADIO_SM_TRM_03_SEND_POWER_UP,
    RADIO_SM_TRM_04_SEND_INTERRUPT_CONFIGURATION,
    RADIO_SM_TRM_05_SEND_FAST_RESPONSE_CONFIGURATION,
    RADIO_SM_TRM_06_SEND_CRYSTAL_TUNING_CAPACITANCE,
    RADIO_SM_TRM_07_SEND_EASY_CONFIG_ARRAY_PART_1,
    RADIO_SM_TRM_08_SEND_NOP,
    RADIO_SM_TRM_09_SEND_EASY_CONFIG_ARRAY_PART_2,
    RADIO_SM_TRM_10_SEND_EASY_CONFIG_ARRAY_CRC_CHECK,
    RADIO_SM_TRM_11_SEND_RESPONSE_EASY_CONFIG_ARRAY_CRC_CHECK,
    RADIO_SM_TRM_12_SEND_GPIO_NIRQ_SDO_CONFIGURATION,
    RADIO_SM_TRM_13_SEND_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_TRM_14_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_TRM_15_SEND_FILL_TRANSMIT_FIFO,
    RADIO_SM_TRM_16_SEND_TRANSMIT_START,
    RADIO_SM_TRM_17_TRANSMIT_LOOP_DELAY_1_SEC,
    RADIO_SM_TRM_18_SEND_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_TRM_19_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND
} RADIO_STATEMACHINE_TRANSMIT_T;

typedef enum
{
    RADIO_SM_REC_01_POWER_ON_DELAY_1_SEC = 1,
    RADIO_SM_REC_02_POWER_UP_SHUTDOWN_20_USEC,
    RADIO_SM_REC_03_SEND_POWER_UP,
    RADIO_SM_REC_04_SEND_INTERRUPT_CONFIGURATION,
    RADIO_SM_REC_05_SEND_FAST_RESPONSE_CONFIGURATION,
    RADIO_SM_REC_06_SEND_CRYSTAL_TUNING_CAPACITANCE,
    RADIO_SM_REC_07_SEND_EASY_CONFIG_ARRAY_PART_1,
    RADIO_SM_REC_08_SEND_NOP,
    RADIO_SM_REC_09_SEND_EASY_CONFIG_ARRAY_PART_2,
    RADIO_SM_REC_10_SEND_EASY_CONFIG_ARRAY_CRC_CHECK,
    RADIO_SM_REC_11_SEND_RESPONSE_EASY_CONFIG_ARRAY_CRC_CHECK,
    RADIO_SM_REC_12_SEND_GPIO_NIRQ_SDO_CONFIGURATION,
    RADIO_SM_REC_13_SEND_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_REC_14_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_REC_15_WAIT_FOR_RECEIVE_INTERRUPT,
    RADIO_SM_REC_16_SEND_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_REC_17_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS,
    RADIO_SM_REC_18_SEND_READ_RECEIVE_FIFO,
    RADIO_SM_REC_19_SEND_CLEAR_RECEIVE_FIFO,
    RADIO_SM_REC_20_SEND_RESPONSE_CLEAR_RECEIVE_FIFO,
    RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND
} RADIO_STATEMACHINE_RECEIVE_T;

typedef enum
{
    RADIO_TEST_BUTTON1 = 1,
    RADIO_TEST_BUTTON2,
    RADIO_TEST_BUTTON3,
    RADIO_TEST_BUTTON4
} RADIO_TEST_BUTTON_T;

// -----------------------------------------------------------------------------
// Public variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Private variables
// -----------------------------------------------------------------------------

static RADIO_STATEMACHINE_TRANSMIT_T radioTransmitState = RADIO_SM_TRM_01_POWER_ON_DELAY_1_SEC;
static RADIO_STATEMACHINE_TRANSMIT_T radioTransmitStateNext = RADIO_SM_TRM_01_POWER_ON_DELAY_1_SEC;
static int32_t radioTransmitDelay = 0;
static uint8_t radioTransmitBuffer[128] = {0};
static RADIO_STATEMACHINE_RECEIVE_T radioReceiveState = RADIO_SM_REC_01_POWER_ON_DELAY_1_SEC;
static RADIO_STATEMACHINE_RECEIVE_T radioReceiveStateNext = RADIO_SM_REC_01_POWER_ON_DELAY_1_SEC;
static int32_t radioReceiveDelay = 0;
static uint8_t radioReceiveBuffer[128] = {0};
static RADIO_TEST_BUTTON_T radioTestButton = RADIO_TEST_BUTTON1;

// RF_POWER_UP command
static uint8_t RF_POWER_UP[] =
{
    0x02,   // POWER_UP
    0x01,   // NoPatch and PowerUpEzRadioToFunctionalMode
    0x00,   // Reserved
    0x01,   // CrystalFrequencyByte3 30 MHz 0x01C9C380
    0xC9,   // CrystalFrequencyByte2 30 MHz 0x01C9C380
    0xC3,   // CrystalFrequencyByte1 30 MHz 0x01C9C380
    0x80    // CrystalFrequencyByte0 30 MHz 0x01C9C380
};

// RF_INT_CTL_ENABLE_2_TRANSMIT command
static uint8_t RF_INT_CTL_ENABLE_2_TRANSMIT[] =
{
    0x11,   // SET_PROPERTY
    0x01,   // Property Interrupt has 4 bytes
            // INT_CTL_ENABLE
            // INT_CTL_PH_ENABLE
            // INT_CTL_MODEM_ENABLE
            // INT_CTL_CHIP_ENABLE
    0x02,   // Property Interrupt set 2 bytes
    0x00,   // Property Interrupt set first byte at offset 0
    0x01,   // INT_CTL_ENABLE ChipInt=Disabled ModemInt=Disabled PacketHandlerInt=Enabled
    0x20    // INT_CTL_PH_ENABLE PACKET_SENT=Enabled
};

// RF_INT_CTL_ENABLE_2_RECEIVE command
static uint8_t RF_INT_CTL_ENABLE_2_RECEIVE[] =
{
    0x11,   // SET_PROPERTY
    0x01,   // Property Interrupt has 4 bytes
            // INT_CTL_ENABLE
            // INT_CTL_PH_ENABLE
            // INT_CTL_MODEM_ENABLE
            // INT_CTL_CHIP_ENABLE
    0x02,   // Property Interrupt set 2 bytes
    0x00,   // Property Interrupt set first byte at offset 0
    0x01,   // INT_CTL_ENABLE ChipInt=Disabled ModemInt=Disabled PacketHandlerInt=Enabled
    0x18    // INT_CTL_PH_ENABLE PACKET_RX=Enabled CRC_ERROR=Enabled
};

// RF_FRR_CTL_A_MODE_4 command
static uint8_t RF_FRR_CTL_A_MODE_4[] =
{
    0x11,   // SET_PROPERTY
    0x02,   // Property FastResponseRegister has 4 bytes
            // FRR_CTL_A_MODE
            // FRR_CTL_B_MODE
            // FRR_CTL_C_MODE
            // FRR_CTL_D_MODE
    0x04,   // Property FastResponseRegister set 4 bytes
    0x00,   // Property FastResponseRegister set first byte at offset 0
    0x08,   // FRR_CTL_A_MODE ChipStatusInterrupt pending
    0x06,   // FRR_CTL_B_MODE ModemInterrupt pending
    0x04,   // FRR_CTL_C_MODE PacketHandlerInterrupt pending
    0x0A    // FRR_CTL_D_MODE latched RSSI value during reception of last packet
};

// RF_EZCONFIG_XO_TUNE_1 command
static uint8_t RF_EZCONFIG_XO_TUNE_1[] =
{
    0x11,   // SET_PROPERTY
    0x24,   // Property EasyConfiguration has 4 bytes
            // EZCONFIG_MODULATION
            // RESERVED
            // RESERVED
            // EZCONFIG_XO_TUNE
    0x01,   // Property EasyConfiguration set 1 byte
    0x03,   // Property EasyConfiguration set first byte at offset 3
    0x52    // EZCONFIG_XO_TUNE crystal load capacitance is 11 + 82 * 0.07 pF = 16.74 pF
};

// RF_WRITE_TX_FIFO command (EZConfigArrayPart1)
static uint8_t RF_WRITE_TX_FIFO[] =
{
    0x66, 0xD9, 0xD3, 0x60, 0x3A, 0x5C, 0x68, 0xB4, 0xCB, 0xF0, 0x9D, 0x4E, 0xFE, 0x75, 0xAE, 0xD6,
    0x56, 0x03, 0xFB, 0x3D, 0x7B, 0xAF, 0x39, 0x1C, 0xB2, 0x25, 0xFC, 0xFC, 0xB3, 0x47, 0x63, 0xFE,
    0x6B, 0x8D, 0x5F, 0x31, 0xCE, 0x50, 0x6C, 0x60, 0x44, 0x23, 0x42, 0x7C, 0x2E, 0xF2, 0x26, 0xBA,
    0xE5, 0x0B, 0x6C, 0xFE, 0xFC, 0x63, 0xFD, 0x28, 0x0C, 0x4D, 0x70, 0x26, 0xB6, 0x98, 0xA9, 0x99,
    0xFA, 0x2D, 0xF4, 0xF4, 0x50, 0xD5, 0x14, 0x88, 0xE8, 0xE3, 0x1E, 0x2B, 0x9B, 0xA2, 0x6D, 0x87,
    0x5D, 0xDB, 0x16, 0x8F, 0xD1, 0x76, 0xD6, 0x6B, 0x2F, 0x66, 0x96, 0xFB, 0x87, 0xFD, 0xDA, 0x12,
    0x59, 0x2C, 0x35, 0xDF, 0xC3, 0xE0, 0x8E, 0x91, 0x1A, 0xD3, 0x40, 0x35, 0x9C, 0xF7, 0xEA, 0x67,
    0x40, 0xC1
};

// RF_NOP command
static uint8_t RF_NOP[] =
{
    0x00
};

// RF_WRITE_TX_FIFO_1 command (EZConfigArrayPart2)
static uint8_t RF_WRITE_TX_FIFO_1[] =
{
    0x66, 0x4B, 0x7A, 0x06, 0x5B, 0xDA, 0x78, 0x2C, 0xDB, 0xCE, 0x47, 0x6D, 0x7A, 0x74, 0x12, 0x9C,
    0x4F, 0x5C, 0x48, 0xB0, 0xEE, 0x26, 0x99, 0xC2, 0x04, 0xA5, 0x80, 0x5A, 0x07, 0xD7, 0xD6, 0x8B,
    0x6F, 0x46, 0xCE, 0x76, 0x55, 0x08, 0xE0, 0xDE, 0x85, 0x1E, 0x7C, 0xEF, 0x9E, 0x22, 0xAF, 0x88,
    0x1F, 0x49, 0x76, 0x82, 0xC2, 0x87, 0xA1, 0x74, 0x0A, 0x00, 0x18, 0xFF, 0xAC, 0xE4, 0x94, 0x89,
    0xB5, 0x8A, 0xA9, 0x65, 0x5C, 0x98, 0xBA, 0x6C, 0x0A, 0x50, 0xF8, 0x59, 0x31, 0x7B, 0x92, 0xD5,
    0x63, 0xB0, 0xC8, 0xB0, 0xE8, 0xDD, 0x99, 0x3E, 0x23, 0x4C, 0x49, 0xAF, 0xC2, 0x30, 0x44, 0x9C,
    0x4B, 0xFA, 0xC2, 0x33, 0xDB, 0x7D, 0xF3, 0x88, 0xE7, 0x6F, 0x1D, 0x80, 0xBF, 0x80, 0xA9, 0x59
};

// RF_EZCONFIG_CHECK command
static uint8_t RF_EZCONFIG_CHECK[] =
{
    0x19,   // EZCONFIG_CHECK
    0x59,   // Check write of EZConfigArray with CRC MSB
    0x61    // Check write of EZConfigArray with CRC LSB
};

// RF_READ_CMD_BUFF_EZCONFIG_CHECK command
static uint8_t RF_READ_CMD_BUFF_EZCONFIG_CHECK[] =
{
    0x44,   // READ_CMD_BUFF
    0x00,   // Send 0x00 Receive CTS 0xFF
    0x00    // Send 0x00 Receive EZConfigArray is valid 0x00
};

// RF_GPIO_PIN_CFG command
static uint8_t RF_GPIO_PIN_CFG[] =
{
    0x13,   // GPIO_PIN_CFG
    0x01,   // Pin GPIO0 disable pullup and disable input and output drivers
    0x01,   // Pin GPIO1 disable pullup and disable input and output drivers
    0x01,   // Pin GPIO2 disable pullup and disable input and output drivers
    0x01,   // Pin GPIO3 disable pullup and disable input and output drivers
    0x00,   // Pin NIRQ disable pullup and default behavior
    0x00,   // Pin SDO disable pullup and default behavior
    0x00    // Configure the 4 gpio pins with the highest drive strength
};

// RF_GET_INT_STATUS command
static uint8_t RF_GET_INT_STATUS[] =
{
    0x20,   // GET_INT_STATUS
    0x00,   // Clear all PacketHandlerInterrupts
    0x00,   // Clear all ModemInterrupts
    0x00    // Clear all ChipInterrupts
};

// RF_READ_CMD_BUFF_GET_INT_STATUS command
static uint8_t RF_READ_CMD_BUFF_GET_INT_STATUS[] =
{
    0x44,   // READ_CMD_BUFF
    0x00,   // Send 0x00 Receive CTS 0xFF
    0x00,   // Send 0x00 Receive ChipIntPend ModemIntPend PacketHandlerIntPend 3 lsb
    0x00,   // Send 0x00 Receive ChipIntStatus ModemIntStatus PacketHandlerIntStatus 3 lsb
    0x00,   // Send 0x00 Receive PacketHandlerPend (PACKET_SENT_PEND, PACKET_RX_PEND, CRC_ERROR_PEND)
    0x00,   // Send 0x00 Receive PacketHandlerStatus
    0x00,   // Send 0x00 Receive ModemPend
    0x00,   // Send 0x00 Receive ModemStatus
    0x00,   // Send 0x00 Receive ChipPend
    0x00    // Send 0x00 Receive ChipStatus
};

// RF_WRITE_TX_FIFO_BUTTON1 command
static uint8_t RF_WRITE_TX_FIFO_BUTTON1[] =
{
    0x66,   // WRITE_TX_FIFO
    0x42,   // 'B'
    0x55,   // 'U'
    0x54,   // 'T'
    0x54,   // 'T'
    0x4F,   // 'O'
    0x4E,   // 'N'
    0x31    // '1'
};

// RF_WRITE_TX_FIFO_BUTTON2 command
static uint8_t RF_WRITE_TX_FIFO_BUTTON2[] =
{
    0x66,   // WRITE_TX_FIFO
    0x42,   // 'B'
    0x55,   // 'U'
    0x54,   // 'T'
    0x54,   // 'T'
    0x4F,   // 'O'
    0x4E,   // 'N'
    0x32    // '2'
};

// RF_WRITE_TX_FIFO_BUTTON3 command
static uint8_t RF_WRITE_TX_FIFO_BUTTON3[] =
{
    0x66,   // WRITE_TX_FIFO
    0x42,   // 'B'
    0x55,   // 'U'
    0x54,   // 'T'
    0x54,   // 'T'
    0x4F,   // 'O'
    0x4E,   // 'N'
    0x33    // '3'
};

// RF_WRITE_TX_FIFO_BUTTON4 command
static uint8_t RF_WRITE_TX_FIFO_BUTTON4[] =
{
    0x66,   // WRITE_TX_FIFO
    0x42,   // 'B'
    0x55,   // 'U'
    0x54,   // 'T'
    0x54,   // 'T'
    0x4F,   // 'O'
    0x4E,   // 'N'
    0x34    // '4'
};

// RF_START_TX command
static uint8_t RF_START_TX[] =
{
    0x31,   // START_TX
    0x00,   // Channel 0
    0x30,   // After transmission of packet return to state READY
    0x00,   // Fixed 0
    0x07    // NrOfBytes to send from TransmitFifo (max 64 bytes)
};

// RF_READ_RX_FIFO command
static uint8_t RF_READ_RX_FIFO[] =
{
    0x77,   // READ_RX_FIFO
    0x00,   // Send 0x00 Receive 0x42 'B'
    0x00,   // Send 0x00 Receive 0x55 'U'
    0x00,   // Send 0x00 Receive 0x54 'T'
    0x00,   // Send 0x00 Receive 0x54 'T'
    0x00,   // Send 0x00 Receive 0x4F 'O'
    0x00,   // Send 0x00 Receive 0x4E 'N'
    0x00    // Send 0x00 Receive 0x31 '1' or 0x32 '2' or 0x33 '3' or 0x34 '4'
};

// RF_FIFO_INFO command
static uint8_t RF_FIFO_INFO[] =
{
    0x15,   // FIFO_INFO
    0x02    // Reset ReceiveFifo
};

// RF_READ_CMD_BUFF_FIFO_INFO command
static uint8_t  RF_READ_CMD_BUFF_FIFO_INFO[] =
{
    0x44,   // READ_CMD_BUFF
    0x00,   // Send 0x00 Receive CTS 0xFF
    0x00,   // Send 0x00 Receive NrOfBytes in ReceiveFifo
    0x00    // Send 0x00 Receive NrOfBytes in TransmitFifo
};

// -----------------------------------------------------------------------------
// Private function prototypes
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Public functions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Function : RADIO_Init()
// Comment  : Init 4 SI4455 pins RF_SHUTDOWN RF_IRQN RF_GPIO1 RF_GPIO2.
// Params   :
//   Input  : -
//   Output : -
// Return   : -
// -----------------------------------------------------------------------------
void RADIO_Init(void)
{
    // Set private radio variables
    radioTransmitState = RADIO_SM_TRM_01_POWER_ON_DELAY_1_SEC;
    radioTransmitStateNext = RADIO_SM_TRM_01_POWER_ON_DELAY_1_SEC;
    radioTransmitDelay = 0;
    STR_MemSetByte(radioTransmitBuffer, 0x00, sizeof(radioTransmitBuffer));
    radioReceiveState = RADIO_SM_REC_01_POWER_ON_DELAY_1_SEC;
    radioReceiveStateNext = RADIO_SM_REC_01_POWER_ON_DELAY_1_SEC;
    radioReceiveDelay = 0;
    STR_MemSetByte(radioReceiveBuffer, 0x00, sizeof(radioReceiveBuffer));
    radioTestButton = RADIO_TEST_BUTTON1;

    // RF_SHUTDOWN = GPIO18 = LPC824_PIN31 is output and inactive
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, RF_SHUTDOWN);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, RF_SHUTDOWN, RF_SHUTDOWN_INACTIVE);

    // RF_IRQN = GPIO17 = LPC824_PIN32 is input
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, RF_IRQN);

    // RF_GPIO1 = GPIO19 = LPC824_PIN30 is input
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, RF_GPIO1);

    // RF_GPIO2 = GPIO20 = LPC824_PIN29 is input
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, RF_GPIO2);
}

// -----------------------------------------------------------------------------
// Function : RADIO_SM_TransmitUpdate()
// Comment  : Update the statemachine that initializes radio transceiver SI4455
//            and transmits a packet with 7 bytes payload every second.
// Params   :
//   Input  : -
//   Output : -
// Return   : -
// -----------------------------------------------------------------------------
void RADIO_SM_TransmitUpdate(void)
{
    // RadioStatemachineTransmit
    switch (radioTransmitState)
    {
        case RADIO_SM_TRM_01_POWER_ON_DELAY_1_SEC:
            if (radioTransmitDelay < RADIO_DELAY_1_SEC)
            {
                radioTransmitDelay++;
            }
            else
            {
                radioTransmitDelay = 0;
                radioTransmitState = RADIO_SM_TRM_02_POWER_UP_SHUTDOWN_20_USEC;
                radioTransmitStateNext = RADIO_SM_TRM_02_POWER_UP_SHUTDOWN_20_USEC;
            }
            break;

        case RADIO_SM_TRM_02_POWER_UP_SHUTDOWN_20_USEC:
            Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, RF_SHUTDOWN, RF_SHUTDOWN_ACTIVE);
            for (radioTransmitDelay = 0; radioTransmitDelay < RADIO_DELAY_20_USEC; radioTransmitDelay++);
            Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, RF_SHUTDOWN, RF_SHUTDOWN_INACTIVE);
            radioTransmitDelay = 0;
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_03_SEND_POWER_UP;
            break;

        case RADIO_SM_TRM_03_SEND_POWER_UP:
            SPI_TransceiveData(sizeof(RF_POWER_UP), RF_POWER_UP, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_04_SEND_INTERRUPT_CONFIGURATION;
            break;

        case RADIO_SM_TRM_04_SEND_INTERRUPT_CONFIGURATION:
            SPI_TransceiveData(sizeof(RF_INT_CTL_ENABLE_2_TRANSMIT), RF_INT_CTL_ENABLE_2_TRANSMIT, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_05_SEND_FAST_RESPONSE_CONFIGURATION;
            break;

        case RADIO_SM_TRM_05_SEND_FAST_RESPONSE_CONFIGURATION:
            SPI_TransceiveData(sizeof(RF_FRR_CTL_A_MODE_4), RF_FRR_CTL_A_MODE_4, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_06_SEND_CRYSTAL_TUNING_CAPACITANCE;
            break;

        case RADIO_SM_TRM_06_SEND_CRYSTAL_TUNING_CAPACITANCE:
            SPI_TransceiveData(sizeof(RF_EZCONFIG_XO_TUNE_1), RF_EZCONFIG_XO_TUNE_1, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_07_SEND_EASY_CONFIG_ARRAY_PART_1;
            break;

        case RADIO_SM_TRM_07_SEND_EASY_CONFIG_ARRAY_PART_1:
            SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO), RF_WRITE_TX_FIFO, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_08_SEND_NOP;
            break;

        case RADIO_SM_TRM_08_SEND_NOP:
            SPI_TransceiveData(sizeof(RF_NOP), RF_NOP, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_09_SEND_EASY_CONFIG_ARRAY_PART_2;
            break;

        case RADIO_SM_TRM_09_SEND_EASY_CONFIG_ARRAY_PART_2:
            SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO_1), RF_WRITE_TX_FIFO_1, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_10_SEND_EASY_CONFIG_ARRAY_CRC_CHECK;
            break;

        case RADIO_SM_TRM_10_SEND_EASY_CONFIG_ARRAY_CRC_CHECK:
            SPI_TransceiveData(sizeof(RF_EZCONFIG_CHECK), RF_EZCONFIG_CHECK, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_11_SEND_RESPONSE_EASY_CONFIG_ARRAY_CRC_CHECK;
            break;

        case RADIO_SM_TRM_11_SEND_RESPONSE_EASY_CONFIG_ARRAY_CRC_CHECK:
            SPI_TransceiveData(sizeof(RF_READ_CMD_BUFF_EZCONFIG_CHECK), RF_READ_CMD_BUFF_EZCONFIG_CHECK, radioReceiveBuffer);
            if (radioReceiveBuffer[2] == EZ_CONFIG_ARRAY_VALID)
            {
                // EZConfigArray is valid continue SI4455 configuration
                radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
                radioTransmitStateNext = RADIO_SM_TRM_12_SEND_GPIO_NIRQ_SDO_CONFIGURATION;
            }
            else
            {
                // EZConfigArray is invalid restart SI4455 configuration
                radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
                radioTransmitStateNext = RADIO_SM_TRM_01_POWER_ON_DELAY_1_SEC;
            }
            break;

        case RADIO_SM_TRM_12_SEND_GPIO_NIRQ_SDO_CONFIGURATION:
            SPI_TransceiveData(sizeof(RF_GPIO_PIN_CFG), RF_GPIO_PIN_CFG, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_13_SEND_CLEAR_ALL_INTERRUPTS;
            break;

        case RADIO_SM_TRM_13_SEND_CLEAR_ALL_INTERRUPTS:
            SPI_TransceiveData(sizeof(RF_GET_INT_STATUS), RF_GET_INT_STATUS, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_14_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS;
            break;

        case RADIO_SM_TRM_14_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS:
            SPI_TransceiveData(sizeof(RF_READ_CMD_BUFF_GET_INT_STATUS), RF_READ_CMD_BUFF_GET_INT_STATUS, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_15_SEND_FILL_TRANSMIT_FIFO;
            break;

        case RADIO_SM_TRM_15_SEND_FILL_TRANSMIT_FIFO:
            switch (radioTestButton)
            {
                case RADIO_TEST_BUTTON1:
                    SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO_BUTTON1), RF_WRITE_TX_FIFO_BUTTON1, radioReceiveBuffer);
                    radioTestButton = RADIO_TEST_BUTTON4;
                    break;

                case RADIO_TEST_BUTTON2:
                    SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO_BUTTON2), RF_WRITE_TX_FIFO_BUTTON2, radioReceiveBuffer);
                    radioTestButton = RADIO_TEST_BUTTON2;
                    break;

                case RADIO_TEST_BUTTON3:
                    SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO_BUTTON3), RF_WRITE_TX_FIFO_BUTTON3, radioReceiveBuffer);
                    radioTestButton = RADIO_TEST_BUTTON3;
                    break;

                case RADIO_TEST_BUTTON4:
                    SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO_BUTTON4), RF_WRITE_TX_FIFO_BUTTON4, radioReceiveBuffer);
                    radioTestButton = RADIO_TEST_BUTTON1;
                    break;

                default:
                    break;
            }
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_16_SEND_TRANSMIT_START;
            break;

        case RADIO_SM_TRM_16_SEND_TRANSMIT_START:
            SPI_TransceiveData(sizeof(RF_START_TX), RF_START_TX, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_17_TRANSMIT_LOOP_DELAY_1_SEC;
            break;

        case RADIO_SM_TRM_17_TRANSMIT_LOOP_DELAY_1_SEC:
            if (radioTransmitDelay < RADIO_DELAY_1_SEC)
            {
                radioTransmitDelay++;
            }
            else
            {
                radioTransmitDelay = 0;
                radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
                radioTransmitStateNext = RADIO_SM_TRM_18_SEND_CLEAR_ALL_INTERRUPTS;
            }
            break;

        case RADIO_SM_TRM_18_SEND_CLEAR_ALL_INTERRUPTS:
            // Clear PH_PACKET_SENT_PEND interrupt
            SPI_TransceiveData(sizeof(RF_GET_INT_STATUS), RF_GET_INT_STATUS, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_19_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS;
            break;

        case RADIO_SM_TRM_19_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS:
            SPI_TransceiveData(sizeof(RF_READ_CMD_BUFF_GET_INT_STATUS), RF_READ_CMD_BUFF_GET_INT_STATUS, radioReceiveBuffer);
            radioTransmitState = RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND;
            radioTransmitStateNext = RADIO_SM_TRM_15_SEND_FILL_TRANSMIT_FIFO;
            break;

        case RADIO_SM_TRM_20_WAIT_FOR_CLEAR_TO_SEND:
            if (Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, RF_GPIO1) == RF_GPIO1_CTS_ACTIVE)
            {
                radioTransmitState = radioTransmitStateNext;
            }
            break;

        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// Function : RADIO_SM_ReceiveUpdate()
// Comment  : Update the statemachine that initializes radio transceiver SI4455
//            and receives a packet with 7 bytes payload every interrupt.
// Params   :
//   Input  : -
//   Output : -
// Return   : -
// -----------------------------------------------------------------------------
void RADIO_SM_ReceiveUpdate(void)
{
    int32_t result = 0;

    result++;
    result--;

    STR_MemSetByte(radioReceiveBuffer, 0x00, sizeof(radioReceiveBuffer));
    STR_MemSetByte(radioTransmitBuffer, 0x00, sizeof(radioTransmitBuffer));
    radioReceiveBuffer[4] = 0xFF;

    result = STR_MemCmpByte(radioReceiveBuffer, radioTransmitBuffer, 8);

    // RadioStatemachineReceive
    switch (radioReceiveState)
    {
        case RADIO_SM_REC_01_POWER_ON_DELAY_1_SEC:
            if (radioReceiveDelay < RADIO_DELAY_1_SEC)
            {
                radioReceiveDelay++;
            }
            else
            {
                radioReceiveDelay = 0;
                radioReceiveState = RADIO_SM_REC_02_POWER_UP_SHUTDOWN_20_USEC;
                radioReceiveStateNext = RADIO_SM_REC_02_POWER_UP_SHUTDOWN_20_USEC;
            }
            break;

        case RADIO_SM_REC_02_POWER_UP_SHUTDOWN_20_USEC:
            Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, RF_SHUTDOWN, RF_SHUTDOWN_ACTIVE);
            for (radioReceiveDelay = 0; radioReceiveDelay < RADIO_DELAY_20_USEC; radioReceiveDelay++);
            Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, RF_SHUTDOWN, RF_SHUTDOWN_INACTIVE);
            radioReceiveDelay = 0;
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_03_SEND_POWER_UP;
            break;

        case RADIO_SM_REC_03_SEND_POWER_UP:
            SPI_TransceiveData(sizeof(RF_POWER_UP), RF_POWER_UP, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_04_SEND_INTERRUPT_CONFIGURATION;
            break;

        case RADIO_SM_REC_04_SEND_INTERRUPT_CONFIGURATION:
            SPI_TransceiveData(sizeof(RF_INT_CTL_ENABLE_2_RECEIVE), RF_INT_CTL_ENABLE_2_RECEIVE, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_05_SEND_FAST_RESPONSE_CONFIGURATION;
            break;

        case RADIO_SM_REC_05_SEND_FAST_RESPONSE_CONFIGURATION:
            SPI_TransceiveData(sizeof(RF_FRR_CTL_A_MODE_4), RF_FRR_CTL_A_MODE_4, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_06_SEND_CRYSTAL_TUNING_CAPACITANCE;
            break;

        case RADIO_SM_REC_06_SEND_CRYSTAL_TUNING_CAPACITANCE:
            SPI_TransceiveData(sizeof(RF_EZCONFIG_XO_TUNE_1), RF_EZCONFIG_XO_TUNE_1, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_07_SEND_EASY_CONFIG_ARRAY_PART_1;
            break;

        case RADIO_SM_REC_07_SEND_EASY_CONFIG_ARRAY_PART_1:
            SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO), RF_WRITE_TX_FIFO, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_08_SEND_NOP;
            break;

        case RADIO_SM_REC_08_SEND_NOP:
            SPI_TransceiveData(sizeof(RF_NOP), RF_NOP, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_09_SEND_EASY_CONFIG_ARRAY_PART_2;
            break;

        case RADIO_SM_REC_09_SEND_EASY_CONFIG_ARRAY_PART_2:
            SPI_TransceiveData(sizeof(RF_WRITE_TX_FIFO_1), RF_WRITE_TX_FIFO_1, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_10_SEND_EASY_CONFIG_ARRAY_CRC_CHECK;
            break;

        case RADIO_SM_REC_10_SEND_EASY_CONFIG_ARRAY_CRC_CHECK:
            SPI_TransceiveData(sizeof(RF_EZCONFIG_CHECK), RF_EZCONFIG_CHECK, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_11_SEND_RESPONSE_EASY_CONFIG_ARRAY_CRC_CHECK;
            break;

        case RADIO_SM_REC_11_SEND_RESPONSE_EASY_CONFIG_ARRAY_CRC_CHECK:
            SPI_TransceiveData(sizeof(RF_READ_CMD_BUFF_EZCONFIG_CHECK), RF_READ_CMD_BUFF_EZCONFIG_CHECK, radioReceiveBuffer);
            if (radioReceiveBuffer[2] == EZ_CONFIG_ARRAY_VALID)
            {
                // EZConfigArray is valid continue SI4455 configuration
                radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
                radioReceiveStateNext = RADIO_SM_REC_12_SEND_GPIO_NIRQ_SDO_CONFIGURATION;
            }
            else
            {
                // EZConfigArray is invalid restart SI4455 configuration
                radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
                radioReceiveStateNext = RADIO_SM_REC_01_POWER_ON_DELAY_1_SEC;
            }
            break;

        case RADIO_SM_REC_12_SEND_GPIO_NIRQ_SDO_CONFIGURATION:
            SPI_TransceiveData(sizeof(RF_GPIO_PIN_CFG), RF_GPIO_PIN_CFG, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_13_SEND_CLEAR_ALL_INTERRUPTS;
            break;

        case RADIO_SM_REC_13_SEND_CLEAR_ALL_INTERRUPTS:
            SPI_TransceiveData(sizeof(RF_GET_INT_STATUS), RF_GET_INT_STATUS, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_14_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS;
            break;

        case RADIO_SM_REC_14_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS:
            SPI_TransceiveData(sizeof(RF_READ_CMD_BUFF_GET_INT_STATUS), RF_READ_CMD_BUFF_GET_INT_STATUS, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_15_WAIT_FOR_RECEIVE_INTERRUPT;
            break;

        case RADIO_SM_REC_15_WAIT_FOR_RECEIVE_INTERRUPT:
            if (Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, RF_IRQN) == RF_IRQN_ACTIVE)
            {
                radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
                radioReceiveStateNext = RADIO_SM_REC_16_SEND_CLEAR_ALL_INTERRUPTS;
            }
            break;

        case RADIO_SM_REC_16_SEND_CLEAR_ALL_INTERRUPTS:
            // Clear PH_PACKET_RX_PEND interrupt or PH_CRC_ERROR_PEND interrupt
            SPI_TransceiveData(sizeof(RF_GET_INT_STATUS), RF_GET_INT_STATUS, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_17_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS;
            break;

        case RADIO_SM_REC_17_SEND_RESPONSE_CLEAR_ALL_INTERRUPTS:
            SPI_TransceiveData(sizeof(RF_READ_CMD_BUFF_GET_INT_STATUS), RF_READ_CMD_BUFF_GET_INT_STATUS, radioReceiveBuffer);
            if ((radioReceiveBuffer[4] & PH_PACKET_RX_PEND) == PH_PACKET_RX_PEND)
            {
                // Valid packet received
                radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
                radioReceiveStateNext = RADIO_SM_REC_18_SEND_READ_RECEIVE_FIFO;
            }
            else if ((radioReceiveBuffer[4] & PH_CRC_ERROR_PEND) == PH_CRC_ERROR_PEND)
            {
                // Invalid packet received
                radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
                radioReceiveStateNext = RADIO_SM_REC_19_SEND_CLEAR_RECEIVE_FIFO;
            }
            break;

        case RADIO_SM_REC_18_SEND_READ_RECEIVE_FIFO:
            SPI_TransceiveData(sizeof(RF_READ_RX_FIFO), RF_READ_RX_FIFO, radioReceiveBuffer);
            // "BUTTON1" or "BUTTON2" or "BUTTON3" or "BUTTON4" received
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_15_WAIT_FOR_RECEIVE_INTERRUPT;

        case RADIO_SM_REC_19_SEND_CLEAR_RECEIVE_FIFO:
            SPI_TransceiveData(sizeof(RF_FIFO_INFO), RF_FIFO_INFO, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_20_SEND_RESPONSE_CLEAR_RECEIVE_FIFO;
            break;

        case RADIO_SM_REC_20_SEND_RESPONSE_CLEAR_RECEIVE_FIFO:
            SPI_TransceiveData(sizeof(RF_READ_CMD_BUFF_FIFO_INFO), RF_READ_CMD_BUFF_FIFO_INFO, radioReceiveBuffer);
            radioReceiveState = RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND;
            radioReceiveStateNext = RADIO_SM_REC_15_WAIT_FOR_RECEIVE_INTERRUPT;
            break;

        case RADIO_SM_REC_21_WAIT_FOR_CLEAR_TO_SEND:
            if (Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, RF_GPIO1) == RF_GPIO1_CTS_ACTIVE)
            {
                radioReceiveState = radioReceiveStateNext;
            }
            break;

        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// Private functions
// -----------------------------------------------------------------------------
