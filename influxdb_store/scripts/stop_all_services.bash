#!/usr/bin/env bash

set -x
sudo systemctl stop jsk-pr1012-influxdb.service
sudo systemctl stop jsk-pr1040-influxdb.service
sudo systemctl stop jsk-fetch15-influxdb.service
sudo systemctl stop jsk-fetch1075-influxdb.service
set +x
