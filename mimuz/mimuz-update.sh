#!/bin/bash -e

S=~/mimuz-tuch/STM32/source/mimuz-tuch
cp $S/Inc/queue32.h Core/Inc/queue32.h
cp $S/Inc/usbd_midi_if.h Core/Inc/usbd_midi_if.h
cp $S/Src/queue32.c Core/Src/queue32.c
cp $S/Src/usbd_midi_if.c Core/Src/usbd_midi_if.c
cp $S/Middlewares/USBMIDI/Inc/usbd_midi.h Middlewares/USBMIDI/Inc/usbd_midi.h
cp $S/Middlewares/USBMIDI/Src/usbd_midi.c Middlewares/USBMIDI/Src/usbd_midi.c
