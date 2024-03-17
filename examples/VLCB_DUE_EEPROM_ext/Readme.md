# VLCB_DUE_EEPROM_ext

This a VLCB version of CANDUE3EEPROM which expands CANDUE3 to allow for the optional use of external EEPROM

The compiling of code optional for the DUE can be controlled using __SAM3X8E__

This code uses files called CANSAM3X8E.h and CANSAM3X8E.cpp which implement the VLCB interface for the DUE.

These chave now been made into a library VCANSAM38X£ when testing is finished.

These files use the libraries due_can (Version 2.1.6) and can_common(0.3.0). Do not use other versions of due_can.

It does not currently find the correct CANID and this is being investigated.

Further use needs more development of the VLCB Arduino library to include event teaching.

John Fletcher 13/11/2023

5/2/2024 Updated for the latest version of the VLCB Arduino library.




