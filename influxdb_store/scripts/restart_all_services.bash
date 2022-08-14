#!/usr/bin/env bash

set -x
sudo systemctl restart jsk-pr1012-influxdb.service
sudo systemctl restart jsk-pr1040-influxdb.service
sudo systemctl restart jsk-fetch15-influxdb.service
sudo systemctl restart jsk-fetch1075-influxdb.service
set +x
