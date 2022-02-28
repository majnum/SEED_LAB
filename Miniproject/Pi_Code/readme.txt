Miniproject Pi readme

This code uses computer vision and I2C communication to set a motors position and display the setpoint and current motor position to an LCD display.

Marker Detection Scheme:
  Initialize and set PiCamera
  Calibrate by taking 4 pictures and averaging auto AWB values to set AWB values
  Enter infinite loop
  Take picture to find marker
  Convert image to HSV
  Isolate color we are looking for using mask and setting bounds on HSV values
  Filter image to remove noise: blur > opening > closing; all with 8x8 kernel
  Convert filtered image to grayscale
  Throw out pixels that are not bright enough
  Calculate center of pixels that are remaining by taking mean of pixel indexes
  Compares the center calculated to the resolution of camera set earlier to determine which region marker is in
