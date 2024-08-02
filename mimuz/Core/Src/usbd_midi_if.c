/**
  ******************************************************************************
  * @file           : usbd_midi_if.c
  * @brief          :
  ******************************************************************************

    (CC at)2016 by D.F.Mac. @TripArts Music

*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi_if.h"
#include "stm32g4xx_hal.h"
#include "../../../common_stuff/z_ringbuffer.h"

// basic midi rx/tx functions
static uint16_t MIDI_DataRx(uint8_t *msg, uint16_t length);
static uint16_t MIDI_DataTx(uint8_t *msg, uint16_t length);

static ring_buffer rxq;
static char rxqraw[1024];

void (*cbNoteOff)(uint8_t ch, uint8_t note, uint8_t vel);
void (*cbNoteOn)(uint8_t ch, uint8_t note, uint8_t vel);
void (*cbCtlChange)(uint8_t ch, uint8_t num, uint8_t value);
uint16_t (*cbAll)(uint8_t *msg, uint16_t length);

static int checkMidiMessage(uint8_t *pMidi);

USBD_MIDI_ItfTypeDef USBD_Interface_fops_FS =
{
  MIDI_DataRx,
  MIDI_DataTx
};

static int available = 0;
static uint16_t MIDI_DataRx(uint8_t *msg, uint16_t length){
  if((length & 3) == 0)
  {
    if(available < length)
      available = rb_available_to_write(&rxq);
    if(available >= length)
      rb_write_to_buffer(&rxq, -1, msg, length);
  }
  return 0;
}

void sendMidiMessage(uint8_t *msg, uint16_t size){
  if(!(size & 3))
    MIDI_DataTx(msg, size);
}

static uint16_t MIDI_DataTx(uint8_t *msg, uint16_t length){
  uint32_t i = 0;
  while (i < length) {
    APP_Rx_Buffer[APP_Rx_ptr_in] = *(msg + i);
    APP_Rx_ptr_in++;
    i++;
    if (APP_Rx_ptr_in == APP_RX_DATA_SIZE) {
      APP_Rx_ptr_in = 0;
    }
  }
  return USBD_OK;
}

// from mi:muz (Internal)
static int checkMidiMessage(uint8_t *pMidi){
  if(((*(pMidi + 1) & 0xf0)== 0x90)&&(*(pMidi + 3) != 0)){
    return 2;
  }else if(((*(pMidi + 1) & 0xf0)== 0x90)&&(*(pMidi + 3) == 0)){
    return 1;
  }else if((*(pMidi + 1) & 0xf0)== 0x80){
    return 1;
  }else if((*(pMidi + 1) & 0xf0)== 0xb0){
    return 3;
  }else{
    return 0;
  }
}


// from mi:muz (Interface functions)
static uint8_t buffer[4];

void mimuz_init(void){
  if(!rxq.size)
    rb_init_preallocated(&rxq, rxqraw, sizeof(rxqraw));
}

void setHdlAll(uint16_t (*fptr)(uint8_t *msg, uint16_t length))
{
  cbAll = fptr;
  mimuz_init();
}

void setHdlNoteOff(void (*fptr)(uint8_t ch, uint8_t note, uint8_t vel)){
  cbNoteOff = fptr;
  mimuz_init();
}

void setHdlNoteOn(void (*fptr)(uint8_t ch, uint8_t note, uint8_t vel)){
  cbNoteOn = fptr;
  mimuz_init();
}

void setHdlCtlChange(void (*fptr)(uint8_t ch, uint8_t num, uint8_t value)){
  cbCtlChange = fptr;
  mimuz_init();
}

void sendNoteOn(uint8_t ch, uint8_t note, uint8_t vel){
  buffer[0] = 0x09;
  buffer[1] = 0x90 | ch;
  buffer[2] = 0x7f & note;
  buffer[3] = 0x7f & vel;
  sendMidiMessage(buffer,4);
}

void sendNoteOff(uint8_t ch, uint8_t note){
  buffer[0] = 0x08;
  buffer[1] = 0x80 | ch;
  buffer[2] = 0x7f & note;
  buffer[3] = 0;
  sendMidiMessage(buffer,4);
}

void sendCtlChange(uint8_t ch, uint8_t num, uint8_t value){
  buffer[0] = 0x0b;
  buffer[1] = 0xb0 | ch;
  buffer[2] = 0x7f & num;
  buffer[3] = 0x7f & value;
  sendMidiMessage(buffer,4);
}

void processMidiMessage(){
  uint8_t pbuf[4];
  uint8_t kindmessage;
  // Rx
  int available = rb_available_to_read(&rxq);
  if(available > 0){
    int ret = rb_read_from_buffer(&rxq, (char*)pbuf, sizeof(pbuf));
    if(!ret)
    {
    kindmessage = checkMidiMessage(pbuf);
    uint8_t handled = 0;
    if(kindmessage == 1){
      if(cbNoteOff != NULL){
        (*cbNoteOff)(*(pbuf+1)&0x0f,*(pbuf+2)&0x7f,*(pbuf+3)&0x7f);
        handled = 1;
      }
    }else if(kindmessage == 2){
      if(cbNoteOn != NULL){
        (*cbNoteOn)(*(pbuf+1)&0x0f,*(pbuf+2)&0x7f,*(pbuf+3)&0x7f);
        handled = 1;
      }
    }else if(kindmessage == 3){
      if(cbCtlChange != NULL){
        (*cbCtlChange)(*(pbuf+1)&0x0f,*(pbuf+2)&0x7f,*(pbuf+3)&0x7f);
        handled = 1;
      }
    }
    if(!handled && cbAll)
      cbAll(pbuf, 4);
    }
  }
  // Tx
  USBD_MIDI_SendPacket();
}

