/*
 * SoftRX - Interrupt-driven serial decoder for the ESP8266 Arduino core
 * Copyright(c) 2017-2018 of wave++ (Yuri D'Elia)
 * Distributed under GNU LGPL without ANY warranty.
 */

#pragma once
#include <Arduino.h>


#define SRX_RX_BUF    32
#define SRX_RATE      9600

#define SRX_US_PULSE  (1000000L / SRX_RATE)
#define SRX_US_SHIFT  (SRX_US_PULSE / 8)
#define SRX_US_TO_BRK (SRX_US_PULSE * 10)

#define SRX_CC_PULSE  (clockCyclesPerMicrosecond() * SRX_US_PULSE)
#define SRX_CC_SHIFT  (clockCyclesPerMicrosecond() * SRX_US_SHIFT)
#define SRX_CC_TO_BRK (clockCyclesPerMicrosecond() * SRX_US_TO_BRK)

#define SRX_STATE_BRK 1
#define SRX_STATE_OVR 2
#define SRX_STATE_ERR 4


class SoftRX
{
  static int rx_pin;
  static uint8_t rx_buf[SRX_RX_BUF];
  static uint8_t rx_byte;
  static uint8_t rx_bit;
  static volatile uint8_t rx_err;
  static volatile uint8_t rx_size;
  static volatile uint8_t rx_state;

  static void
  _wait_reset();

  static void
  _trigger_reset();

  static void
  _char_handler();

  static void
  _break_handler();


public:
  static void
  begin(int rx_pin);

  static void
  flush();

  static void
  clear();

  static int
  available()
  {
    return rx_size;
  }

  static int
  state()
  {
    return rx_state;
  }

  static int
  errors()
  {
    return rx_err;
  }

  static int
  read(uint8_t* buf, int size);
};

extern SoftRX Serial2;
