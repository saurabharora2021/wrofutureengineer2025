#!/bin/bash

SERVICE_NAME="mywroapp"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"

# Stop the service if running
sudo systemctl stop "$SERVICE_NAME"

# Disable the service
sudo systemctl disable "$SERVICE_NAME"

# Remove the service file
if [ -f "$SERVICE_FILE" ]; then
    sudo rm "$SERVICE_FILE"
    echo "Removed $SERVICE_FILE"
fi

# Reload systemd to apply changes
sudo systemctl daemon-reload

echo "Service $SERVICE_NAME uninstalled."