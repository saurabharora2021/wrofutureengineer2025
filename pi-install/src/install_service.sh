#!/bin/bash

# Variables
SERVICE_NAME="mywroapp"
PROJECT_DIR="/home/piwro/wrofutureengineer2025/wroprg"
VENV_DIR="$PROJECT_DIR/.venv"
PYTHON_SCRIPT="$PROJECT_DIR/src/main.py"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"
PIGPIOD_SERVICE_FILE="/etc/systemd/system/pigpiod_custom.service"

# Create virtual environment if not exists
if [ ! -d "$VENV_DIR" ]; then
    sudo -u piwro python3 -m venv "$VENV_DIR"
fi

# Install requirements
sudo apt-get install -y i2c-tools python3-pip python3-venv python3-smbus pigpio

sudo systemctl enable pigpiod 2>/dev/null || true

sudo -u piwro "$VENV_DIR/bin/pip" install -r "$PROJECT_DIR/requirements.txt"

# Create main app systemd service file, require pigpiod
sudo tee "$SERVICE_FILE" > /dev/null <<EOL
[Unit]
Description=My Python App Service
After=network.target pigpiod.service
Requires=pigpiod.service
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

# Reload systemd, enable and start services
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

echo "Services $SERVICE_NAME and pigpiod_custom installed and started."
