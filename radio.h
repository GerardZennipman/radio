// -----------------------------------------------------------------------------
// File     : radio.h
// Product  : RF_V1
// Created  : 10-Mar-2018
// Author   : Gerard Zennipman
// Comment  : This file contains SI4455 radio transmit and receive functions.
// -----------------------------------------------------------------------------

#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "chip.h"

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
// Public functions
// -----------------------------------------------------------------------------

void RADIO_Init(void);
void RADIO_SM_TransmitUpdate(void);
void RADIO_SM_ReceiveUpdate(void);
int32_t RADIO_IsTransmitterAvailable(void);
void RADIO_TransmitString(char *string);

#endif // RADIO_H
