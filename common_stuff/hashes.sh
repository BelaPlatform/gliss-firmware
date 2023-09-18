#!/bin/bash
set -e
p=$(dirname $0)
cd $p

FILE=gitHashes.h
HASHES=
SEP=
for a in . TrillRackApplication TrillRackApplication/trill-neopixel TrillRackApplication/psoc-programmer-stm32; do
	HASHES=$HASHES$SEP$(git -C ../$a describe --always --abbrev=10 --dirty)
	SEP=-
done
STRING="#define GIT_HASHES \"$HASHES\""
diff <(echo $STRING) $FILE 2>/dev/null || echo $STRING > $FILE
echo $HASHES
