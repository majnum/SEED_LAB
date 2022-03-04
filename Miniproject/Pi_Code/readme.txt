Miniproject Pi readme

Authors for Pi Code: Dylan Bott, Joshua Higgins

This code uses computer vision and I2C communication to set a motors position and display the setpoint and current motor position to an LCD display.

The final code being used was CombinedCode.py. The pictures currently in the repository aren't great examples of the pictures the PiCamera had taken, but there are the calibration pictures used to set the AWB gains and markerPic.jpg is the picture containing our marker that gets filtered and returns the region the marker is in.

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
