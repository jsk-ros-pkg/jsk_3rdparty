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

exec rosrun julius julius $ARGS
