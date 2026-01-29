# RC transmitter
RC transmitter with 1 to 7 channels.
Includes nRF24L01+ transceiver and ATmega328P processor.
Telemetry monitors receiver voltage using LED indication.
The code is Arduino.

This RC transmitter works with RC receiver from my repository [**RX_nRF24_Motor_Servo**](https://github.com/stanekTM/RX_nRF24_Motor_Servo)

## The firmware includes
### LED mode:
* Normal mode, LED is lit
* If the TX battery is low, the LED blink at 0.5s interval
* If the RX battery is low, the LED blink at 0.3s interval
* If we lose RF data for 1 second, the LED blink at 0.1s interval
### Calibration:
* Hold calibration button, switch transmitter TX on, still holding calibration button move all controls to extremes.
* Center all controls.
* Release calibration button (saved to eeprom).
### Servo reversing:
* To reverse the desired channel, hold the joystick in the end position and turn on the TX transmitter (saved to eeprom).

## Arduino pins
```
Pots:
Number of channels according to control elements (possible combination, max 7)
A0 to A6

D4 - calibration button
D6 - LED
A7 - input TX battery

nRF24L01:
D9  - CE
D10 - CSN
D11 - MOSI
D12 - MISO
D13 - SCK
```
