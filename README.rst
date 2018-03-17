Low-latency software serial for the ESP8266 Arduino core
========================================================

This projects adds an interrupt-driven ``Serial2`` object to the
`ESP8266 Arduino Core <https://github.com/esp8266/Arduino/>`_, allowing
efficient high-speed serial decoding on any available GPIO pin.

Combining ``Serial2`` (read-only) with the existing `Serial1
<https://arduino-esp8266.readthedocs.io/en/latest/reference.html#serial>`_
(write-only), you can effectively have a second full-duplex low-latency
serial port on the ESP8266 which is much more efficient than
SoftwareSerial_. ``Serial2`` is self-synchronizing and supports
``BREAK`` detection just like any hardware UART decoder.

Usage
-----

Customize the decoding speed and buffer size directly in ``softrx.hpp``.
Usage of the ``Serial2`` object is similar to the normal ``Serial``
object.

After including ``"softrx.hpp"`` itself, initialize the decoder with the
correct GPIO pin using ``begin()``. Use ``Serial2.available()`` and
``Serial2.read()`` to consume any input:

.. code:: cpp

   #include <Arduino.h>
   #include "softrx.hpp"

   #define RX_PIN D4

   void setup()
   {
     Serial2.begin(RX_PIN);
   }

   void loop()
   {
      char buf[4];
      if(Serial2.available() >= sizeof(buf))
      {
	Serial2.read(buf, sizeof(buf));
      }
   }

``Serial2.state()`` returns the current decoder state as a bitmask. The
following bits can be set:

:SRX_STATE_BRK: Serial ``BREAK``
:SRX_STATE_OVR: Buffer overflow
:SRX_STATE_ERR: Decoder error

``Serial2.errors()`` returns the cumulative error count since the last
``Serial2.clear()`` call.


Caveats
-------

``Serial2`` takes control of ``TIMER1``, which can be a big limitation
depending on your project. You can only have one additional serial
decoder (``Serial2`` itself).

You cannot use any ``TIMER1`` functionality once ``begin()`` has been
called. A notable example using ``TIMER1`` is the PWM functionality
distributed in the ESP8266 core itself via ``analogWrite()``.

Revert to ``SoftwareSerial`` if ``TIMER1`` is needed anywhere else in
your project.

.. _SoftwareSerial: https://github.com/plerup/espsoftwareserial


Authors and Copyright
---------------------

| `SoftRX` is distributed under LGPLv2+ (see COPYING) WITHOUT ANY WARRANTY.
| Copyright(c) 2017-2018 by wave++ "Yuri D'Elia" <wavexx@thregr.org>.
