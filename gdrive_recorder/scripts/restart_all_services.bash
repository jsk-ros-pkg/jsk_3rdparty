#!/usr/bin/env bash

set -x
# sudo systemctl restart jsk-pr1012-gdrive.service
# sudo systemctl restart jsk-pr1040-gdrive.service
# sudo systemctl restart jsk-fetch15-gdrive.service
# sudo systemctl restart jsk-fetch1075-gdrive.service
sudo systemctl restart jsk-pr1012-gdrive-recorder.service
sudo systemctl restart jsk-pr1040-gdrive-recorder.service
sudo systemctl restart jsk-fetch15-gdrive-recorder.service
sudo systemctl restart jsk-fetch1075-gdrive-recorder.service
set +x
