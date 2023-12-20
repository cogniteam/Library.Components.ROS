#!/bin/bash

SLEEP=${1:-1}
COMMAND="${@:2}"


sleep ${SLEEP}

bash -i -c "${COMMAND}"