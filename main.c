// -----------------------------------------------------------------------------
// File     : main.c
// Product  : RF_V1
// Created  : 10-Mar-2018
// Author   : Gerard Zennipman
// Comment  : This file contains main() which is the entry point after reset.
// -----------------------------------------------------------------------------

#include <stdint.h>
#include "chip.h"
#include "systemtick.h"
#include "led.h"
#include "stringfunctions.h"
#include "uartrf.h"
#include "debug.h"
#include "adc.h"
#include "switch.h"
#include "test.h"
#include "spi.h"
#include "radio.h"

// -----------------------------------------------------------------------------
// Constant definitions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Type definitions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Public variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Private variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Private function prototypes
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Public functions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Function : main()
// Comment  : The reset interrupt handler initializes ram variables by
//            copying their initial value from flash or sets them to 0.
//            Then the reset interrupt handler turns on the iocon clock
//            and gpio clock and initializes the system clock.
//            Then the reset interrupt handler calls main().
// Params   :
//   Input  : -
//   Output : -
// Return   : -
// -----------------------------------------------------------------------------
int main(void)
{
    int32_t swVersion = 19;
    int32_t redLed = 0;
    int32_t greenLed = 0;
    int32_t debugValue = 0;
    int32_t adcRFVoltage = 0;
    int32_t openSwitch = 0;
    int32_t closeSwitch = 0;
    int32_t testSCL = 1;
    int32_t testSCLOld = 1;
    int32_t testSDA = 1;
    int32_t testSDAOld = 1;
    char transmitString[80] = {0};
    char tmpString[80] = {0};
    char receivedString[80] = {0};

    // Interrupts and priorities:
    // - Specify interrupts in VectorTable in cr_startup_lpc82x.c
    //   - SYS_SystemTick_IRQHandler()
    //   - UART_RF_IRQHandler()
    // - Global CortexM0+ disable interrupts with configurable priority
    // - Local Peripherals enable interrupts with configurable priority
    // - Global CortexM0+ enable interrupts with configurable priority

    // Global CortexM0+ disable interrupts with configurable priority
    __disable_irq();

    // Set public variable SystemCoreClock
    SystemCoreClockUpdate();

    // Set red green leds to output and off
    LED_Init();

    // Initialize RfUart0 to 115200 8+no+1
    UART_RF_Init();

    // Set debug pin DBG_GPIO to output and off
    DBG_Init();

    // Initialize analog to digital converter
    // ADC0 = ADC_VRF_12V
    ADC_Init();

    // Set open close switch pins to output and off
    SWITCH_Init();

    // Initialize 2 input pins TEST_I2C_SCL and TEST_I2C_SDA
    TEST_Init();

    // Initialize SPI0 and clock data on positive edge with 1 Mbps
    SPI_Init();

    // Initialize radio SI4455
    RADIO_Init();

    // Local Peripheral enable interrupt with configurable priority
    // Set SYS_SystemTick_IRQHandler() priority to 3 (0=hi 3=lo), SCB->SHP
    // Enable SYS_SystemTick_IRQHandler(), SysTick->CTRL
    SYS_StartSystemTick1MilliSecond();

    // Local Peripheral enable interrupt with configurable priority
    // Set UART_RF_IRQHandler() priority to 0 (0=hi 3=lo), NVIC->IP
    // Enable UART_RF_IRQHandler(), NVIC->ISER
    NVIC_SetPriority(UART0_IRQn, 0);
    NVIC_EnableIRQ(UART0_IRQn);

    // Global CortexM0+ enable interrupts with configurable priority
    __enable_irq();

    // Loop forever
    while (1)
    {
        // Every msec:
        // - Use last adc sample to calculate real world value
        // - Read red green leds
        // - Read SCL SDA
        // - Update LedStateMachine that controls red green leds
        // - Update RadioStateMachineTransmit or RadioStateMachineReceive
        // - Check and handle change of pin SCL and SDA
        // - Check and handle received uart command
        if (SYS_GetTimer1MilliSeconds() == 0)
        {
            SYS_SetTimer1MilliSeconds(1);

            // ADC_CalculateRealWorldValue() takes 10 usec
            ADC_CalculateRealWorldValue();

            redLed = LED_GetRedLed();
            greenLed = LED_GetGreenLed();

            testSCL = TEST_GetSCL();
            testSDA = TEST_GetSDA();

            LED_SM_Update();

            // RADIO_SM_TransmitUpdate();
            RADIO_SM_ReceiveUpdate();

            // If SCL or SDA change then transmit corresponding radio packet
            if (RADIO_IsTransmitterAvailable() == 1)
            {
                if (testSCL != testSCLOld)
                {
                    if (testSCL == 0)
                    {
                        // SCL is pressed
                        RADIO_TransmitString("OPEN_1");
                        // RADIO_TransmitString("BUTTON1");
                    }
                    else
                    {
                        // SCL is released
                        RADIO_TransmitString("OPEN_0");
                        // RADIO_TransmitString("BUTTON2");
                    }
                }
                if (testSDA != testSDAOld)
                {
                    if (testSDA == 0)
                    {
                        // SDA is pressed
                        RADIO_TransmitString("CLOSE_1");
                        // RADIO_TransmitString("BUTTON3");
                    }
                    else
                    {
                        // SDA is released
                        RADIO_TransmitString("CLOSE_0");
                        // RADIO_TransmitString("BUTTON4");
                    }
                }
            }
            testSCLOld = testSCL;
            testSDAOld = testSDA;

            // If RfUart0 has received string that ends with "\r\n" then copy it
            if (UART_RF_CheckReceivedString(receivedString) == 1)
            {
                // Received string is e.g. "get_sw_version\r\n"
                if (STR_StrCmp("get_sw_version", receivedString) == 1)
                {
                    if (UART_RF_IsTransmitterAvailable() == 1)
                    {
                        // Transmit "SoftwareVersion: xx\r\n"
                        STR_StrCpy("SoftwareVersion: ", transmitString);
                        STR_IToA(swVersion, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_StrCat("\r\n", transmitString);
                        UART_RF_TransmitString(transmitString);
                    }
                }

                // Received string is e.g. "get_rg_leds\r\n"
                if (STR_StrCmp("get_rg_leds", receivedString) == 1)
                {
                    if (UART_RF_IsTransmitterAvailable() == 1)
                    {
                        // Transmit "RGLeds: rg\r\n"
                        STR_StrCpy("RGLeds: ", transmitString);
                        STR_IToA(redLed, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_IToA(greenLed, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_StrCat("\r\n", transmitString);
                        UART_RF_TransmitString(transmitString);
                    }
                }

                // Received string is e.g. "set_dbg_gpio 1\r\n"
                if (STR_StrCmp("set_dbg_gpio", receivedString) == 1)
                {
                    debugValue = STR_StrFindNr(receivedString);
                    if (debugValue == 1)
                    {
                        DBG_SetDbgGpio(1);
                    }
                    else
                    {
                        DBG_SetDbgGpio(0);
                    }
                }

                // Received string is e.g. "get_dbg_gpio\r\n"
                if (STR_StrCmp("get_dbg_gpio", receivedString) == 1)
                {
                    if (UART_RF_IsTransmitterAvailable() == 1)
                    {
                        // Transmit "DBG_GPIO: 1/0\r\n"
                        STR_StrCpy("DBG_GPIO: ", transmitString);
                        debugValue = DBG_GetDbgGpio();
                        STR_IToA(debugValue, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_StrCat("\r\n", transmitString);
                        UART_RF_TransmitString(transmitString);
                    }
                }

                // Received string is e.g. "set_dbgvar1 1\r\n"
                if (STR_StrCmp("set_dbgvar1", receivedString) == 1)
                {
                    debugValue = STR_StrFindNr(receivedString);
                    DBG_SetDbgVar1(debugValue);
                }

                // Received string is e.g. "get_dbgvar1\r\n"
                if (STR_StrCmp("get_dbgvar1", receivedString) == 1)
                {
                    if (UART_RF_IsTransmitterAvailable() == 1)
                    {
                        // Transmit "DbgVar1: xx\r\n"
                        STR_StrCpy("DbgVar1: ", transmitString);
                        debugValue = DBG_GetDbgVar1();
                        STR_IToA(debugValue, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_StrCat("\r\n", transmitString);
                        UART_RF_TransmitString(transmitString);
                    }
                }

                // Received string is e.g. "get_rf_voltage\r\n"
                if (STR_StrCmp("get_rf_voltage", receivedString) == 1)
                {
                    if (UART_RF_IsTransmitterAvailable() == 1)
                    {
                        // Transmit "RFVoltage: xx\r\n"
                        STR_StrCpy("RFVoltage: ", transmitString);
                        adcRFVoltage = ADC_GetRFVoltage();
                        STR_IToA(adcRFVoltage, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_StrCat("\r\n", transmitString);
                        UART_RF_TransmitString(transmitString);
                    }
                }

                // Received string is e.g. "set_switch_open 1\r\n"
                if (STR_StrCmp("set_switch_open", receivedString) == 1)
                {
                    openSwitch = STR_StrFindNr(receivedString);
                    SWITCH_SetOpenSwitch(openSwitch);
                }

                // Received string is e.g. "set_switch_close 1\r\n"
                if (STR_StrCmp("set_switch_close", receivedString) == 1)
                {
                    closeSwitch = STR_StrFindNr(receivedString);
                    SWITCH_SetCloseSwitch(closeSwitch);
                }

                // Received string is e.g. "get_scl\r\n"
                if (STR_StrCmp("get_scl", receivedString) == 1)
                {
                    if (UART_RF_IsTransmitterAvailable() == 1)
                    {
                        // Transmit "SCL: 1/0\r\n"
                        STR_StrCpy("SCL: ", transmitString);
                        STR_IToA(testSCL, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_StrCat("\r\n", transmitString);
                        UART_RF_TransmitString(transmitString);
                    }
                }

                // Received string is e.g. "get_sda\r\n"
                if (STR_StrCmp("get_sda", receivedString) == 1)
                {
                    if (UART_RF_IsTransmitterAvailable() == 1)
                    {
                        // Transmit "SDA: 1/0\r\n"
                        STR_StrCpy("SDA: ", transmitString);
                        STR_IToA(testSDA, tmpString);
                        STR_StrCat(tmpString, transmitString);
                        STR_StrCat("\r\n", transmitString);
                        UART_RF_TransmitString(transmitString);
                    }
                }
            }
        }

        // Wait for interrupt from SystemTick or other source
        __WFI();
    }
}

// -----------------------------------------------------------------------------
// Private functions
// -----------------------------------------------------------------------------
