#!/usr/bin/env Rscript
cat(paste(installed.packages()[,c(1)],collapse=";"))
