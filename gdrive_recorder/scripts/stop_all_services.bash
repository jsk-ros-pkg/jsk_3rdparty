#!/usr/bin/env bash

set -x
# sudo systemctl stop jsk-pr1012-gdrive.service
# sudo systemctl stop jsk-pr1040-gdrive.service
# sudo systemctl stop jsk-fetch15-gdrive.service
# sudo systemctl stop jsk-fetch1075-gdrive.service
sudo systemctl stop jsk-pr1012-gdrive-recorder.service
sudo systemctl stop jsk-pr1040-gdrive-recorder.service
sudo systemctl stop jsk-fetch15-gdrive-recorder.service
sudo systemctl stop jsk-fetch1075-gdrive-recorder.service
set +x
