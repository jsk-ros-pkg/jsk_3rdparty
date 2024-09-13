#!/bin/bash

# lock by pid
# PIDFILE=/var/run/mongodump_backup_to_qnap.pid

if [ "$1" == "" ]; then
    echo "Please specify the date (i.e. 20221116)";
    exit 1;
else
    DATE=$1;
    ORIG_DIR="/mnt/hdd3/data/${DATE}_influxdb_backup";
fi

if [ ! -d "$ORIG_DIR" ]; then
    echo "Dir not exists: $ORIG_DIR";
    exit 1;
else 
    echo "copying from $ORIG_DIR to /mnt/qnap"
fi

if mount | awk '{if ($3 == "/mnt/qnap") {exit 0}} ENDFILE{exit -1}'; then
    echo "/mnt/qnap is mounted"
else
    echo "/mnt/qnap is not mounted"
    exit 1
fi


PIDFILE=/var/run/rsync_backup_hdd3_to_qnap.pid
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
exec rsync --inplace -auvhP $ORIG_DIR /mnt/qnap
date
echo "backup ends."
rm -f $PIDFILE
