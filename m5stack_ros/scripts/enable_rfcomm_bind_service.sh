#!/bin/bash

# rfcomm_bind.service should be launched by root
# So this config file is placed under /etc/systemd/system

SYSTEMD_SYSTEM_DIR=/etc/systemd/system

function enable_rfcomm_bind_service () {
    echo "Copy rfcomm_bind.service to $SYSTEMD_SYSTEM_DIR"
    sudo cp $(rospack find m5stack_ros)/config/rfcomm_bind.service $SYSTEMD_SYSTEM_DIR

    echo "Enable rfcomm_bind.service autostart"
    sudo systemctl enable rfcomm_bind.service
}

function disable_rfcomm_bind_service () {
    echo "Disable rfcomm_bind.service autostart"
    sudo systemctl disable rfcomm_bind.service

    echo "rm $SYSTEMD_SYSTEM_DIR/rfcomm_bind.service"
    sudo rm $SYSTEMD_SYSTEM_DIR
}

enable_rfcomm_bind_service
