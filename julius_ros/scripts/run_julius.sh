#!/bin/bash

ARGS=""
for OPT in "$@"; do
  case "$OPT" in
    __*)  # remove ROS arguments
      shift
      ;;
    *)
      ARGS="$ARGS $1"
      shift
      ;;
  esac
done

if [ "$ROS_DISTRO" = "debian" ]; then
    if ! test -e $(rospack find julius)/conf -a -e $(rospack find julius)/model; then
	echo "julius need install grammer and dictation kit, run 'rosrun julius download_julius_data.sh'"
	exit 1
    fi
    exec julius $ARGS
else
    exec rosrun julius julius $ARGS
fi
