#!/bin/bash

PID_FILE="/tmp/recording.pid"

if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    echo "Stopping recording process (PID: $PID)..."
    kill "$PID"
    rm "$PID_FILE"
    echo "Recording stopped."
else
    echo "No active recording found."
fi