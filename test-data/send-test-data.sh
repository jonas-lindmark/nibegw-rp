#!/usr/bin/env bash

set -e

BAUD=9600
PORT=$1
while true; do
  for filename in *.bin; do
    echo "Sending $filename"
    serialcat -b $BAUD "$PORT" -i "$filename"
  done
done