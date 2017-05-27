# May Music Box

This is a little project I made for my daughter. It's an
mp3 player that reads off an SD-card that's built to
replace the guts of a Munchkin Mozart Music Cube.

Why do this? Cause electronic beepy toys for toddlers
are sooo much better if you can control what gets played
on them. :)

## Features
  * Super low power draw. Inactive, draws around 210uA. Active ~50-60mA. With a 1200mAH battery, that's ~8 months of sleep (not really since the battery will leak), or 20 hours of play on one charge.
  * Hardware button debounce. Cuz I could.
  * LEDs that act ike Peak-meter to give your toddler the authentic club feel.
  * Way too much love and sweat put into buliding this thing.
  * The word "mayberry" is printed on the PCB. :)

## What's in this repository
  * Arduino sketch for the MCU that handles the input buttons and playback.
  * vdsp4 assembly code for handling mono downmix, and LED pulsing in of the VS1053b chip
  * Eagle files for the contol PCB.

## Quirks
  * There is a fork of the Adafruit vs1053b library. The mainline library has (multiple) synchronization bugs that lead to corrupted state. This fork fixes the major ones. Remove once upstream merges the fixes.
