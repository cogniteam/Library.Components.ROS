#!/bin/sh

PORT=$1
DIR=$2

/usr/local/bin/http-server -p $PORT $DIR || /usr/bin/http-server -p $PORT $DIR