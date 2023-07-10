#!/bin/bash
set -e

bootloader_k=8
flasher_k=56
application_k=320

dd if=../Bootloader/G491KE-TrillRack.bin of=bootloader-full.bin count=$bootloader_k bs=1k
dd if=../Flasher/G491KE-TrillRack.bin of=fl-full.bin count=$bootloader_k bs=1k
dd if=../Debug/G491KE-TrillRack.bin of=application-full.bin count=$application_k bs=1k

name=G491KE-TrillRack.bin
cp ../Bootloader/$name .
dd if=../Flasher/$name of=$name oseek=$bootloader_k iseek=0 bs=1k
dd if=../Debug/$name of=$name oseek=$((bootloader_k+flasher_k)) iseek=0 bs=1k
