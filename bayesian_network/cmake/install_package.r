#!/usr/bin/env Rscript
args <- commandArgs(TRUE)
res <- try(install.packages(args, repos="http://cran.us.r-project.org"))
if(inherits(res, "try-error")) q(status=1) else q()
