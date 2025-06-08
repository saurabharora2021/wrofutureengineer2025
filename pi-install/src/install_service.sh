#!/bin/bash

# Variables
SERVICE_NAME="mywroapp"
PROJECT_DIR="/home/piwro/wrofutureengineer2025/wroprg"
VENV_DIR="$PROJECT_DIR/.venv"
PYTHON_SCRIPT="$PROJECT_DIR/src/main.py"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"

# Create virtual environment if not exists
if [ ! -d "$VENV_DIR" ]; then
    sudo -u piwro python3 -m venv "$VENV_DIR"
fi

# Install requirements
sudo -u piwro "$VENV_DIR/bin/pip" install -r "$PROJECT_DIR/requirements.txt"

# Create systemd service file
sudo tee "$SERVICE_FILE" > /dev/null <<EOL
[Unit]
Description=My Python App Service
After=network.target
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
User=piwro
WorkingDirectory=/home/piwro/wrofutureengineer2025/wroprg/src
ExecStart=/home/piwro/wrofutureengineer2025/wroprg/.venv/bin/python /home/piwro/wrofutureengineer2025/wroprg/src/main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOL

# Reload systemd, enable and start service
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

echo "Service $SERVICE_NAME installed and started."