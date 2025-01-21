#!/bin/bash
cd "$(dirname "$0")"

#copy the service file into the system
sudo cp ./uwb-reload.service /etc/systemd/system/

#enable and activate the service
sudo systemctl enable uwb-reload.service
sudo systemctl start uwb-reload.service

#verify
sudo systemctl status uwb-reload.service
