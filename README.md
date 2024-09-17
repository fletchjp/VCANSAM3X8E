<img align="right" src="ArduinoVLCB.png"  width="150" height="75">

# VCANSAM3X8E

## Library for VLCB use on an Arduino DUE

This library provides a software CAN interface on an Arduino DUE when used in conjunction with the VLCB_Arduino library
suite.  The main VLCB-Arduino library can be found at [VLCB](https://github.com/SvenRosvall/VLCB-Arduino) 

This VLCB library code is based on a [CBUS library](https://github.com/MERG-DEV/CBUSSAM3X8E) created by Duncan Greenwood
and extended by members of [MERG](https://www.merg.org.uk/). See below under Credits.
This code has been adapted for use with VLCB.

See [Design documents](https://github.com/SvenRosvall/VLCB-Arduino/blob/main/docs/Design.md) for how this library is structured.

## Dependencies
To be added

## Hardware

This library is designed to provide use of VLCB on the Arduino DUE which has no EEPROM installed.

There is provision for the use of EEPROM simulation and as an alternative to use external EEPROM.

## Getting help and support

At the moment this library is still in development and thus not fully supported.

If you have any questions or suggestions please contact the library maintainers
by email to [martindc.merg@gmail.com](mailto:martindc.merg@gmail.com) or create an issue in GitHub.

## Credits

* Duncan Greenwood - Created the CBUS library for Arduinos upon which this VLCB library is based on.
* Sven Rosvall - Converted the CBUS library into the main VLCB-Arduino library suite.
* Martin Da Costa - Contributed to Sven's work and converted the CBUSACAN2040 library to make VCAN2040.
* John Fletcher - Adapted the CBUSSAM3X8E library for VLCB using VCAN2040 as a model.

## License

The code contained in this repository and the executable distributions are licensed under the terms of the
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](LICENSE.md).

If you have questions about licensing this library please contact [martindc.merg@gmail.com](mailto:martindc.merg@gmail.com)
