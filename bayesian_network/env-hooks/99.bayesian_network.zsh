#!/bin/bash

if [ "@DEVELSPACE@" = "True" -o "@DEVELSPACE@" = "true" ]; then
  export R_LIBS=@CATKIN_DEVEL_PREFIX@/lib/R/site-library:$R_LIBS
else
  export R_LIBS=@CATKIN_INSTALL_PREFIX@/lib/R/site-library:$R_LIBS
fi
