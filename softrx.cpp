/*
 * SoftRX - Interrupt-driven serial decoder for the ESP8266 Arduino core
 * Copyright(c) 2017-2018 of wave++ (Yuri D'Elia)
 * Distributed under GNU LGPL without ANY warranty.
 */

#include "softrx.hpp"


// static storage
int SoftRX::rx_pin;
uint8_t SoftRX::rx_buf[SRX_RX_BUF];
uint8_t SoftRX::rx_byte;
uint8_t SoftRX::rx_bit;
volatile uint8_t SoftRX::rx_err;
volatile uint8_t SoftRX::rx_size;
volatile uint8_t SoftRX::rx_state;

// the main object
SoftRX Serial2;


// definitions
class NoInterrupts
{
  uint32_t savedPS;

public:
  NoInterrupts()
  { savedPS = xt_rsil(15); }

  ~NoInterrupts()
  { xt_wsr_ps(savedPS); }
};


void
SoftRX::_wait_reset()
{
  // wait for BREAK or TRIGGER
  attachInterrupt(digitalPinToInterrupt(rx_pin), _trigger_reset, FALLING);
  timer1_disable();
  timer1_attachInterrupt(_break_handler);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
  timer1_write(SRX_CC_TO_BRK);
}


void
SoftRX::_trigger_reset()
{
  // transition to CHAR state
  detachInterrupt(digitalPinToInterrupt(rx_pin));
  rx_state &= ~SRX_STATE_BRK;
  rx_byte = 0;
  rx_bit = 0;

  // shift to the middle of the pulse
  timer1_disable();
  timer1_attachInterrupt(_char_handler);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
  timer1_write(SRX_CC_SHIFT);
}


void
SoftRX::_char_handler()
{
  int v = digitalRead(rx_pin);

  if(rx_bit > 0 && rx_bit < 9)
    rx_byte |= (v << (rx_bit - 1));
  else if(rx_bit == 0)
  {
    if(v == 1)
    {
      rx_err += 1;
      rx_state |= SRX_STATE_ERR;
      _wait_reset();
    }
    else
    {
      timer1_disable();
      timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
      timer1_write(SRX_CC_PULSE);
    }
  }
  else if(rx_bit == 9)
  {
    if(v == 1)
    {
      if(rx_size < SRX_RX_BUF)
	rx_buf[rx_size++] = rx_byte;
      else
      {
	rx_err += 1;
	rx_state |= SRX_STATE_OVR;
      }
    }
    else
    {
      rx_err += 1;
      rx_state |= SRX_STATE_ERR;
    }
    _wait_reset();
  }

  rx_bit += 1;
}


void
SoftRX::_break_handler()
{
  rx_state |= SRX_STATE_BRK;
}


void
SoftRX::begin(int rx_pin)
{
  SoftRX::rx_pin = rx_pin;
  rx_state = 0;
  flush();
  pinMode(rx_pin, INPUT);
  timer1_isr_init();
  _wait_reset();
}


void
SoftRX::flush()
{
  NoInterrupts lock;
  rx_state &= ~SRX_STATE_OVR;
  rx_size = 0;
}


void
SoftRX::clear()
{
  NoInterrupts lock;
  rx_state &= ~(SRX_STATE_ERR | SRX_STATE_OVR);
  rx_size = 0;
  rx_err = 0;
}


int
SoftRX::read(uint8_t* buf, int size)
{
  int read;
  NoInterrupts lock;

  if(size < rx_size)
  {
    read = size;
    rx_size -= size;
    memcpy(buf, rx_buf, read);
    memmove(rx_buf, rx_buf+size, rx_size);
  }
  else
  {
    read = rx_size;
    rx_size = 0;
    memcpy(buf, rx_buf, read);
  }

  if(read)
    rx_state &= ~SRX_STATE_OVR;

  return read;
}
