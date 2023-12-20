#!/bin/sh

mkdir burn_ws
cd burn_ws
git clone $1
ls > ../type.txt
type=$( cat ../type.txt )
cd $type

if [ "$type" = "elf" ]; then 
    arm-none-eabi-objcopy -O binary $2 burn.bin
    st-flash write burn.bin 0x8000000
else
    st-flash write $2 0x8000000
fi

cd ../..
rm -r -f burn_ws 
rm type.txt
