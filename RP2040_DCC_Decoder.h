/* 
 * RP2040_DCC_Decoder.h
 * RP2040 (Raspberry Pico) DCC decoder module
 * Copyright (c) - 2022/2023 - Benoit BOUCHEZ (M8718)
 */
 
 #ifndef __RP2040_DCC_DECODER_H__
 #define __RP2040_DCC_DECODER_H__
 
 #include "pico/stdlib.h"
 
 // Adapt this value 
#ifndef DCC_INPUT_PIN
//#define DCC_INPUT_PIN			16
#define DCC_INPUT_PIN			3
#endif

//! Maximum DCC packet size including extensions (not yet defined in the norm) and extended address
// 2 address bytes + 5 instruction bytes + error detection byte (longest DCC instruction is 3 bytes)
#define MAX_DCC_PACKET_SIZE		8		

//! Initialize and start DCC decoder for background interrupt processing
void StartDCCDecoder (void);

//! Get last received DCC packet
//! \return 0 if no new DCC packet received, otherwise return the size of received packet
unsigned int GetDCCPacket (uint8_t* RXBuffer);
 
 #endif