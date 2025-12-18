#!/bin/bash

# Detect the real user (if sudo was used, use SUDO_USER, otherwise use USER)
REAL_USER=${SUDO_USER:-$USER}

# Get that user's home directory (e.g., /home/group7)
REAL_HOME=$(getent passwd $REAL_USER | cut -d: -f6)

echo "Starting Docker for user: $REAL_USER at $REAL_HOME"

# Run docker-compose passing the variables inline
# This OVERRIDES any .env file settings
USER=$REAL_USER HOME=$REAL_HOME DISPLAY=$DISPLAY docker-compose up