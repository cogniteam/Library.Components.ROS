#!/bin/sh
arch=$(uname -m)
if [ "$arch" = "aarch64" ]; then 
    pip install https://github.com/alphacep/vosk-api/releases/download/v0.3.31/vosk-0.3.31-py3-none-linux_aarch64.whl
else
    pip3 install vosk
fi