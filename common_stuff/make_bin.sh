#!/bin/bash
set -ex

p=$(dirname $0)
cd $p

bootloader_k=8
flasher_k=56
application_k=320
name=G491KE-TrillRack.bin

for a in Bootloader Flasher Debug; do
	make -C ../$a $name
done

cp ../Bootloader/$name .
dd if=../Flasher/$name of=$name oseek=$bootloader_k iseek=0 bs=1k
dd if=../Debug/$name of=$name oseek=$((bootloader_k+flasher_k)) iseek=0 bs=1k
GIT_ARGS="rev-parse --short=10 HEAD"
echo $(git -C .. $GIT_ARGS)-$(git -C ../TrillRackApplication $GIT_ARGS)-$(git -C ../TrillRackApplication/trill-neopixel $GIT_ARGS)
