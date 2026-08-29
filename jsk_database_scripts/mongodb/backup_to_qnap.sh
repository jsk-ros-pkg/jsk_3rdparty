#!/bin/bash

# lock by pid
# PIDFILE=/var/run/mongodump_backup_to_qnap.pid
#
if mount | awk '{if ($3 == "/media/qnap") {exit 0}} ENDFILE{exit -1}'; then
    echo "/media/qnap is mounted"
else
    echo "/media/qnap is not mounted"
    exit 1
fi

PIDFILE=/var/run/rsync_backup_to_qnap.pid
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
exec rsync --inplace -auvhP /media/mongo2/mongodb_store /media/qnap/mongodb_store_dump/
date
echo "backup ends."
rm -f $PIDFILE
