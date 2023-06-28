#!/bin/bash
set -e

bootloader_k=64
application_k=320

dd if=../Bootloader/G491KE-TrillRack.bin of=bootloader-full.bin count=$bootloader_k bs=1k && bin2h bootloader-full.bin | tail -n +4 > hexdumped_bootloader.h
dd if=../Debug/G491KE-TrillRack.bin of=application-full.bin count=$application_k bs=1k iseek=$bootloader_k && bin2h application-full.bin | tail -n +4 > hexdumped_application.h

hexdump -C ../Bootloader/G491KE-TrillRack.bin  > boot
hexdump -C ../Debug/G491KE-TrillRack.bin  > deb
