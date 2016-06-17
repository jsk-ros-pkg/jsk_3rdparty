#!/bin/bash

@[if DEVELSPACE ]@
  export R_LIBS="@(CATKIN_DEVEL_PREFIX)/lib/R/site-library:$R_LIBS"
@[else]@
  export R_LIBS="@(CMAKE_INSTALL_PREFIX)/lib/R/site-library:$R_LIBS"
@[end if]@
