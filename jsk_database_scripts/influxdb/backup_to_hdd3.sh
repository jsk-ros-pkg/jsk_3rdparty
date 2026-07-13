#!/bin/bash

# lock by pid
# PIDFILE=/var/run/mongodump_backup_to_qnap.pid
PIDFILE=/var/run/rsync_backup_to_hdd3.pid
if [ -f $PIDFILE ]; then
    PID=$(cat $PIDFILE)
    ps -p $PID > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "Process already running"
        exit 1
    else
        # forgot to remove PIDFILE?
        echo $$ > $PIDFILE
        if [ $? -ne 0 ]; then
            echo "Could not create PID file at $PIDFILE"
            exit 1
        fi
    fi
else
    # no running process
    echo $$ > $PIDFILE
    if [ $? -ne 0 ]; then
        echo "Could not create PID file at $PIDFILE"
        exit 1
    fi
fi

date
echo "backup starts."
exec rsync --inplace -auvhP /mnt/hdd2/data/influxdb /mnt/hdd3/data/current_influxdb_backup
date
echo "backup ends."
rm -f $PIDFILE
