#!/usr/bin/env bash

set -x
sudo systemctl start jsk-pr1012-influxdb.service
sudo systemctl start jsk-pr1040-influxdb.service
sudo systemctl start jsk-fetch15-influxdb.service
sudo systemctl start jsk-fetch1075-influxdb.service
set +x
