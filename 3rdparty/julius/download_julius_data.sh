#!/bin/bash -xe

SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
MD5SUM_DIR=${SCRIPT_DIR}/md5sum
INSTALL_DIR=$(rospack find julius)

DIR=$(mktemp -d /tmp/julius-XXXX)
cd $DIR
make -f ${SCRIPT_DIR}/Makefile.dictation-kit MD5SUM_DIR=${MD5SUM_DIR} INSTALL_DIR=${INSTALL_DIR}
make -f ${SCRIPT_DIR}/Makefile.grammar-kit MD5SUM_DIR=${MD5SUM_DIR} INSTALL_DIR=${INSTALL_DIR}
rm -fr $DIR
