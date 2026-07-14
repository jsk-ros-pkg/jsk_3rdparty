#!/bin/bash

# m5stack_ros.service should be launched by user (not root)
# So this config file is placed under $HOME/.config/systemd/user

SYSTEMD_USER_DIR=$HOME/.config/systemd/user

function enable_m5stack_ros_service () {
    if [ ! -d $SYSTEMD_USER_DIR ]; then
        echo "Create directory $SYSTEMD_USER_DIR"
        mkdir -p $SYSTEMD_USER_DIR
    else
        echo "$SYSTEMD_USER_DIR already exists"
    fi

    echo "Copy m5stack_ros.service to $SYSTEMD_USER_DIR"
    cp $(rospack find m5stack_ros)/config/m5stack_ros.service $SYSTEMD_USER_DIR

    echo "Enable m5stack_ros.service autostart"
    systemctl --user enable m5stack_ros.service

    echo "Start m5stack_ros.service at boot (user login is no longer needed)"
    sudo loginctl enable-linger $USER
}

function disable_m5stack_ros_service () {
    echo "Disable m5stack_ros.service autostart"
    systemctl --user disable m5stack_ros.service

    echo "rm $SYSTEMD_USER_DIR/m5stack_ros.service"
    rm $SYSTEMD_USER_DIR/m5stack_ros.service

    echo "Do not start m5stack_ros.service at boot (user login is needed)"
    sudo loginctl disable-linger $USER
}

enable_m5stack_ros_service
