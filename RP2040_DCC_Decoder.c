/* 
 * RP2040_DCC_Decoder.c
 * RP2040 (Raspberry Pico) DCC decoder module
 * Copyright (c) - 2022/2023 - Benoit BOUCHEZ (M8718)
 */
 
#include "RP2040_DCC_Decoder.h"
#include <string.h>
#include "hardware/gpio.h"
 
// Decoder state machine
#define STATE_IDLE			0			// Waiting for preamble
#define STATE_PREAMBLE		1			// Receiving preamble
#define STATE_WAIT_START	2			// Waiting for Packet Start Bit
#define STATE_RECEIVE_BYTE	3			// Receiving DCC packet byte
#define STATE_END_BYTE		4			// Waiting bit after byte to know if DCC packet is finished or not

// Variables used by interrupt to reconstruct incoming DCC packet
static absolute_time_t falling_edge_time;
static absolute_time_t rising_edge_time;
static unsigned int DecoderState = STATE_IDLE;
static unsigned int PreambleBitCounter = 0;
static unsigned int ReceivedBitCounter = 0;						// Count received bits from DCC for the current byte
static unsigned int ReceivedByteCounter = 0;					// Count bytes from incoming DCC packet
static uint8_t ReceivedByte;									// Byte being received from DCC
static uint8_t IncomingDCCPacket[MAX_DCC_PACKET_SIZE];			// DCC packet being reconstructed by decoder
					
// Intermediate buffer for DCC packet sent to application					
static uint8_t LastDCCPacket[MAX_DCC_PACKET_SIZE];
static unsigned int LastDCCPacketSize;
static uint8_t NewDCCPacketAvailable = 0;

// Functions prototypes for the interrupts
void track_signal_rise(unsigned int gpio, long unsigned int events);
void track_signal_fall(unsigned int gpio, long unsigned int events);

void track_signal_rise(unsigned int gpio, long unsigned int events) 
{
    rising_edge_time = get_absolute_time();
	// Make the RP2040 wait until falling edge on DCC input
    gpio_set_irq_enabled_with_callback(DCC_INPUT_PIN, GPIO_IRQ_EDGE_RISE, false, &track_signal_rise);
    gpio_set_irq_enabled_with_callback(DCC_INPUT_PIN, GPIO_IRQ_EDGE_FALL, true, &track_signal_fall);
}  // track_signal_rise
//---------------------------------------------------------------------------

void track_signal_fall(unsigned int gpio, long unsigned int events) 
{
	unsigned int NewBit;
	
    falling_edge_time = get_absolute_time();
    int64_t time_logical_high  = absolute_time_diff_us(rising_edge_time,falling_edge_time);
	
    if(time_logical_high > 87) 
		NewBit = 0;
	else
		NewBit = 1;
	
	// Decode incoming bit
	switch (DecoderState)
	{
		case STATE_IDLE :
			if (NewBit!=0)
			{
				DecoderState = STATE_PREAMBLE;
				PreambleBitCounter = 1;
			}
			break;
		case STATE_PREAMBLE :
			// Count '1' bits to check preamble
			if (NewBit == 0)		// Not enough bits for the preamble : restart decoder
				DecoderState = STATE_IDLE;
			else
			{
				PreambleBitCounter++;
				if (PreambleBitCounter>=10)
				{
					DecoderState = STATE_WAIT_START;;
				}
			}
			break;
		case STATE_WAIT_START :
			// Wait until we receive Packet Start Bit (0)
			if (NewBit == 0)
			{
				DecoderState = STATE_RECEIVE_BYTE;
				ReceivedBitCounter = 0;
				ReceivedByteCounter = 0;
			}
			break;
		case STATE_RECEIVE_BYTE :
			ReceivedByte = ReceivedByte<<1;
			if (NewBit)
				ReceivedByte |= 1;
			ReceivedBitCounter++;
		
			if (ReceivedBitCounter==8)
			{
				// Byte has been fully received : store it in array and when for next bit
				ReceivedBitCounter = 0;
				IncomingDCCPacket[ReceivedByteCounter] = ReceivedByte;
				if (ReceivedByteCounter<MAX_DCC_PACKET_SIZE)
					ReceivedByteCounter++;
				DecoderState = STATE_END_BYTE;
			}
			break;
		case STATE_END_BYTE :
			// Store received byte into message array
			if (NewBit==0)
			{  // This is a start bit for a new byte
				DecoderState = STATE_RECEIVE_BYTE;
			}
			else
			{  // DCC packet is finished : send DCC packet to application and reset state machine
				DecoderState = STATE_IDLE;
			
				if (ReceivedByteCounter>0)
				{
					if (NewDCCPacketAvailable==0)	// Make sure the application has released the buffer before we copy a new content in it
					{
						memcpy (&LastDCCPacket[0], &IncomingDCCPacket[0], ReceivedByteCounter);
						LastDCCPacketSize = ReceivedByteCounter;
						NewDCCPacketAvailable = 1;
					}
				}
			}
			break;
		default :
			// Reset decoder state if we reach an unknown state (this should never happen...)
			DecoderState = STATE_IDLE;
	}
	
	// Make the RP2040 wait for rising edge of DCC input
    gpio_set_irq_enabled_with_callback(DCC_INPUT_PIN, GPIO_IRQ_EDGE_FALL, false, &track_signal_fall);
    gpio_set_irq_enabled_with_callback(DCC_INPUT_PIN, GPIO_IRQ_EDGE_RISE, true, &track_signal_rise);
}  // track_signal_fall
//---------------------------------------------------------------------------

void StartDCCDecoder (void)
{
	gpio_init(DCC_INPUT_PIN);
    gpio_set_dir(DCC_INPUT_PIN, GPIO_IN);
	gpio_pull_up(DCC_INPUT_PIN);
	// Make the RP2040 wait for rising edge on DCC input
	gpio_set_irq_enabled_with_callback(DCC_INPUT_PIN, GPIO_IRQ_EDGE_RISE, true, &track_signal_rise);	
}  // StartDCCDecoder
//---------------------------------------------------------------------------

unsigned int GetDCCPacket (uint8_t* RXBuffer)
{
	unsigned int PacketSize;
	
	if (NewDCCPacketAvailable==0) return 0;
	
	PacketSize = LastDCCPacketSize;
	memcpy (RXBuffer, &LastDCCPacket[0], PacketSize);
	NewDCCPacketAvailable = 0;		// Tell the decoder we are ready to process another packet
	
	return PacketSize;
}  // GetDCCPacket
//---------------------------------------------------------------------------
