/* 
 * DCCLocoDecoder_main.c
 * RP2040 / Raspberry Pico based DCC locomotive decoder
 * Copyright (c) - 2022/2023 - Benoit BOUCHEZ (M8718)
 * V0.1
 */
 
extern "C"{
#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <tusb.h>
};

#include "RP2040_DCC_Decoder.h"

// GPIO definition
#define LED_PIN          25
// NOTE : the two PWM channels must be in the same slice to make the decoder works !
#define PWM_FORWARD_PIN     9
#define PWM_REVERSE_PIN     8

#define FUNC_A_PIN        14
#define FUNC_B_PIN        15
#define FUNC_C_PIN        16
#define FUNC_D_PIN        17

bool CurrentDirectionForward;
uint8_t CurrentSpeed;
uint8_t DecoderAddress = 10;
uint16_t FunctionData = 0;

//! Table to translate 14 / 28 speed step into 128 speed step
uint8_t SpeedTranslate1428[32] = {0, 0, 0, 0, 5, 9, 14, 18, 23, 27, 32, 36, 41, 45, 50, 54, 59, 64, 68, 73, 77, 82, 86, 91, 95, 100, 104, 109, 113, 118, 122, 127};

//! Configure the PWM generators for Forward and Reverse generators
void ConfigurePWM (void)
{
  unsigned int SliceNum;
  pwm_config config;
  
  gpio_set_dir (PWM_FORWARD_PIN, GPIO_OUT);
  gpio_set_dir (PWM_REVERSE_PIN, GPIO_OUT);
  
  // Source frequency is 125MHz by default (divider is 1 at reset)
  // We use a wrap of 4096 to get a PWM frequency of 125 000 000 / 4096 = 30.5kHz
  // as the BD6232 needs a switching frequency between 20kHz and 100kHz
  
  // Both PWM generators are in slice xx (GP  IO 8 and GPIO 9) so we need
  // to configure only this slice
  SliceNum = pwm_gpio_to_slice_num(PWM_REVERSE_PIN);
  
  config = pwm_get_default_config();
  pwm_config_set_wrap(&config, 4096);
    pwm_init (SliceNum, &config, true);   // This enable PWM output : set it to false by default
  
  //pwm_set_gpio_level(PWM_FORWARD_PIN, 0);
  //pwm_set_gpio_level(PWM_REVERSE_PIN, 0);
  pwm_set_chan_level (SliceNum, PWM_CHAN_A, 0);
  pwm_set_chan_level (SliceNum, PWM_CHAN_B, 0);
}  // ConfigurePWM
//---------------------------------------------------------------------------

void SetForwardDirection (void)
{
  unsigned int SliceNum;
  
  SliceNum = pwm_gpio_to_slice_num(PWM_REVERSE_PIN);

  // Set PWM to 0 so motor stops
  pwm_set_chan_level (SliceNum, PWM_CHAN_A, 0);
  pwm_set_chan_level (SliceNum, PWM_CHAN_B, 0); 
  
  // Set reverse pin to standard GPIO to stop PWM production
  // and set it to low level to make BD6232 run into PWM B mode
  gpio_set_function (PWM_REVERSE_PIN, GPIO_FUNC_SIO);
  gpio_put (PWM_REVERSE_PIN, 0);
  
  // Reconnect the forward pin to PWM generator
    gpio_set_function (PWM_FORWARD_PIN, GPIO_FUNC_PWM);
  
  CurrentDirectionForward = true;
}  // SetForwardDirection
//---------------------------------------------------------------------------

void SetReverseDirection (void)
{
  unsigned int SliceNum;
  
  SliceNum = pwm_gpio_to_slice_num(PWM_REVERSE_PIN);  
  
  // Set PWM to 0 so motor stops
  pwm_set_chan_level (SliceNum, PWM_CHAN_A, 0);
  pwm_set_chan_level (SliceNum, PWM_CHAN_B, 0); 
  
  // Set forward pin to standard GPIO to stop PWM production
  // and set it to low level to make BD6232 run into PWM B mode
  gpio_set_function (PWM_FORWARD_PIN, GPIO_FUNC_SIO);
  gpio_put (PWM_FORWARD_PIN, 0);
  
  // Reconnect the reverse pin to PWM generator
    gpio_set_function (PWM_REVERSE_PIN, GPIO_FUNC_PWM); 
  
  CurrentDirectionForward = false;
}  // SetReverseDirection
//---------------------------------------------------------------------------

void SetMotorSpeed (uint8_t DCCSpeed)
{
  // Maximum speed setpoint from DCC is 127, but PWM is 4095
  // So lets' scale our speed for PWM...
  unsigned int PWMFlip = (unsigned int)DCCSpeed<<5;
  
  if (CurrentDirectionForward)
  {
    pwm_set_chan_level (pwm_gpio_to_slice_num(PWM_FORWARD_PIN), PWM_CHAN_B, PWMFlip);
  }
  else
  {
    pwm_set_chan_level (pwm_gpio_to_slice_num(PWM_REVERSE_PIN), PWM_CHAN_A, PWMFlip);
  }
  
  CurrentSpeed = DCCSpeed;
}  // SetMotorSpeed
//---------------------------------------------------------------------------

void InterpretPacket (void)
{
  uint8_t ReceivedPacket[MAX_DCC_PACKET_SIZE];
  unsigned int PacketSize;
  uint8_t DCCSpeed;
  bool DCCDirectionForward;
  uint16_t NewFunctionData;
  uint8_t InstructionCode;
  
  PacketSize = GetDCCPacket (&ReceivedPacket[0]);
    
  // Decode DCC packet and check if it is for us
  if (PacketSize>0)
  {
    // First byte must be our address
    if (ReceivedPacket[0]==DecoderAddress)      // Only short address for now
    {
      // Check error detection byte
      // TODO
      
      Serial.printf ("%d\n", PacketSize);
  
      if (PacketSize == 3)
      {
        NewFunctionData = FunctionData;
        InstructionCode = ReceivedPacket[1]&0xE0; 
        
        if (InstructionCode == 0x80)
        { // Function Group One instruction
          if ((ReceivedPacket[1] & 0x10)==0)
          { // set FL = 0, control F4..F1
            NewFunctionData &= 0xFFE0;    // Clear FO to F4
            NewFunctionData |= (ReceivedPacket[1]&0x0F)<<1;   // Put bits 1 to 4 into function word (F0 is in bit 5)
          }
          else
          { // set FL = 1, control F4..F1
            NewFunctionData &= 0xFFE0;    // Clear FO to F4         
            NewFunctionData |= 0x0001;    // Set F0
            NewFunctionData |= (ReceivedPacket[1]&0x0F)<<1;   // Put bits 1 to 4 into function word
          }
        }
        else if (InstructionCode == 0xA0)
        { // Function Group Two instruction
          if ((ReceivedPacket[1] & 0x10)==0)
          { // Control F12..F9
            NewFunctionData &= 0xE1FF;    
            NewFunctionData |= (ReceivedPacket[1]&0x0F)<<9;         }
          else          
          { // Control F8..F5
            NewFunctionData &= 0xFE1F;    
            NewFunctionData |= (ReceivedPacket[1]&0x0F)<<5;
          }
        }
        else if (InstructionCode == 0x40)
        {   // Reverse direction speed control
          if (CurrentDirectionForward)
          { // TODO : this has to be done using ramps
            SetReverseDirection ();
          }
          
          // Process emergency stop
          DCCSpeed = ReceivedPacket[1]&0x1F;
          if ((DCCSpeed == 0x01)||(DCCSpeed == 0x11))
          {
            SetMotorSpeed (0);    // TODO : reset ramp
          }
          else
          {
            SetMotorSpeed (SpeedTranslate1428[DCCSpeed]);
          }
        }
        else if (InstructionCode == 0x60)
        { // Forward direction speed control
          if (CurrentDirectionForward == false)
          { // TODO : this has to be done using ramps
            SetForwardDirection ();
          }
          
          // Process emergency stop
          DCCSpeed = ReceivedPacket[1]&0x1F;
          if ((DCCSpeed == 0x01)||(DCCSpeed == 0x11))
          {
            SetMotorSpeed (0);    // TODO : reset ramp
          }
          else
          {
            SetMotorSpeed (SpeedTranslate1428[DCCSpeed]);
          }
        }

        FunctionData = NewFunctionData;
      }  // Packet size = 3
      
      //  Advanced Operation Instruction : 128 speed step control
      else if ((ReceivedPacket[1]==0x3F)&&(PacketSize=4))
      {
        DCCSpeed = ReceivedPacket[2]&0x7F;
        DCCDirectionForward = ((ReceivedPacket[2]&0x80)!=0);
        
        // Check if command station has asked for a direction change
        if (DCCDirectionForward != CurrentDirectionForward)
        {   // TODO : this has to be done using ramps
          if (DCCDirectionForward) SetForwardDirection();
          else SetReverseDirection ();
        }
        
        // If we have a new speed, call update PWM
        // TODO : implement acceleration/deceleration ramps
        if (CurrentSpeed != DCCSpeed)
        {
          if (DCCSpeed==1)
          {
            SetMotorSpeed (0);    // Emergency stop : TODO reset ramps
          }
          SetMotorSpeed (DCCSpeed);
        }       
      }  // 128 speed step control
    }
  }
}  // InterpretPacket
//---------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  
  gpio_init(LED_PIN);
  gpio_set_dir (LED_PIN, GPIO_OUT);
  
  gpio_init (FUNC_A_PIN);
  gpio_init (FUNC_B_PIN);
  gpio_init (FUNC_C_PIN);
  gpio_init (FUNC_D_PIN);
  gpio_set_dir (FUNC_A_PIN, GPIO_OUT);
  gpio_set_dir (FUNC_B_PIN, GPIO_OUT);
  gpio_set_dir (FUNC_C_PIN, GPIO_OUT);
  gpio_set_dir (FUNC_D_PIN, GPIO_OUT);
  gpio_pull_down (FUNC_A_PIN);
  gpio_pull_down (FUNC_B_PIN);
  gpio_pull_down (FUNC_C_PIN);
  gpio_pull_down (FUNC_D_PIN);
    
  // Wait here until USB communication is ready
  // TODO : remove after debugging
  while(!Serial || millis() < 5000UL); // hangs here 5 seconds if no USB

  // Activate LED to show USB is connected    
  gpio_put (LED_PIN, 1);
  
  ConfigurePWM();
  SetForwardDirection();
  SetMotorSpeed (0);
      
  StartDCCDecoder ();
}

void loop()
{
  InterpretPacket ();
  
  // Control function outputs depending on Function Control Word
  // FUNC_A is on when F0=1 and forward direction, 0 when F0=0
  if ((FunctionData&0x0001)!=0)
  {  // FL activated
    if (CurrentDirectionForward) 
      gpio_put (FUNC_A_PIN, 1);
    else 
      gpio_put (FUNC_A_PIN, 0);
  }
  else gpio_put (FUNC_A_PIN, 0);
  
  // FUNC_B is on when F0=1 and reverse direction, 0 when F0=0
  if ((FunctionData&0x0001)!=0)
  {  // FL activated
    if (CurrentDirectionForward) 
      gpio_put (FUNC_B_PIN, 0);
    else 
      gpio_put (FUNC_B_PIN, 1); 
  }
  else gpio_put (FUNC_B_PIN, 0);
  
  // FUNC_C is controlled by F1
  gpio_put (FUNC_C_PIN, ((FunctionData&0x0002)!=0));
  
  // FUNC_D is controlled by F2
  gpio_put (FUNC_D_PIN, ((FunctionData&0x0004)!=0));    
}
