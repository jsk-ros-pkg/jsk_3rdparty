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
    print_error() {
	local message="$1"
	echo -e "\e[1m\e[31m${message}\e[0m" >&2
    }
    if ! test -e $(rospack find julius)/conf -a -e $(rospack find julius)/model; then
	print_error "[${BASH_SOURCE[0]}] ----"
	print_error "[${BASH_SOURCE[0]}] ----"
	print_error "[${BASH_SOURCE[0]}] ---- julius need install grammer and dictation kit, run 'sudo PATH=\$PATH LD_LIBRARY_PATH=\$LD_LIBRARY_PATH ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH  $(rospack find julius)/download_julius_data.sh'"
	print_error "[${BASH_SOURCE[0]}] ----"
	print_error "[${BASH_SOURCE[0]}] ----"
	exit 1
    fi
    if ! command -v julius >/dev/null 2>&1; then
	print_error "[${BASH_SOURCE[0]}] ----"
	print_error "[${BASH_SOURCE[0]}] ----"
	print_error "[${BASH_SOURCE[0]}] ---- executable file 'julius' not found, run 'sudo apt install julius'"
	print_error "[${BASH_SOURCE[0]}] ----"
	print_error "[${BASH_SOURCE[0]}] ----"
	exit 1
    fi
    exec julius $ARGS
else
    exec rosrun julius julius $ARGS
fi
