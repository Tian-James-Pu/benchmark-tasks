# Fly escape detector: parts

Prices were checked in November 2026 (US$, single units). Equivalent parts work: any 5 V Arduino with I2C, one PWM pin and 17 other free pins (pin numbers in detector.ino),
any VL53L1X breakout with a regulator and level-shifted I2C, any logic-level N-MOSFET.

| # | Qty | Part | Link | Price each |
|---|---|---|---|---|
| 1 | 1 | Arduino Nano (the sketch uses A6 and A7, which an Uno lacks) | https://store.arduino.cc/products/arduino-nano | about $25 |
| 2 | 13 | Pololu #3415 VL53L1X time-of-flight distance sensor carrier with voltage regulator (27 degree field of view, 4 cm minimum accurate range, short mode to about 130 cm; 13 x 18 mm) | https://www.pololu.com/product/3415 | $22.95 |
| 3 | 1 | Adafruit #412 small push-pull solenoid, 12 V DC, 5.5 mm stroke, 40 ohm (about 300 mA), with return spring | https://www.adafruit.com/product/412 | $7.50 |
| 4 | 1 | Adafruit #355 N-channel power MOSFET IRLB8721 (logic level, 30 V) | https://www.adafruit.com/product/355 | $2.25 |
| 5 | 1 | 1N4001 diode (flyback across the solenoid), Adafruit #755 | https://www.adafruit.com/product/755 | $1.50 (10-pack) |
| 6 | 1 | 12 V 1 A regulated power supply, 5.5/2.1 mm barrel, Adafruit #798, plus a panel barrel jack | https://www.adafruit.com/product/798 | $8.95 |
| 7 | 1 | Red LED, green LED, 2 x 330 ohm, 220 ohm, 100 k and 2 x 10 k resistors; 12 mm tactile push button (RESET); 2-pin header and jumper (CAL) | any | about $3 |
| 8 | 1 | Library: Pololu VL53L1X for Arduino (Library Manager: "VL53L1X" by Pololu) | https://github.com/pololu/vl53l1x-arduino | free |
| 9 | 1 | Sensor puck: 127 mm (5 in) across, 12 mm thick, PLA print or MDF disc, with the pockets in WIRING.md | 127 mm (5 in) hole saw for the panel | about $5 in material |
| 10 | 1 | Fly pad: 7 mm black wool felt cut to a fly's outline (7 x 4 mm, 3 mm thick), glued to a 3 mm x 20 mm steel or brass rod pressed onto the solenoid's plunger | craft felt, hobby rod | about $2 |
| 11 | 1 | Felt ring: 14 mm outside, 8 mm inside, 1 mm thick, glued flush round the centre hole | craft felt | - |
| 12 | 2 | Washable stamp pads in two colours (one for the fly pad, one for the felt ring) | any | about $5 |
| 13 | 1 | 12 mm MDF backer block 80 x 80 mm behind the puck, holding the solenoid; hook-up wire, perfboard | any | about $5 |

The total is about $360, of which the 13 sensors are $298. Unbranded VL53L1X breakouts at about $6 each work too, if
they have a regulator and level-shifted I2C, and bring the total to about $140.

Calibration needs a steel rule (300 mm), a protractor, a 100 x 100 mm piece of white card (300 gsm) taped to a 30 cm
wooden stick, and a phone that films at 240 fps (`CALIBRATION.md`).
