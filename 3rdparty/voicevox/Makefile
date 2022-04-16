# -*- makefile -*-

all: installed.voicevox

VERSION = 0.11.4
FILENAME = onnxruntime-linux-x64-1.10.0.tgz
TARBALL = build/$(FILENAME)
TARBALL_URL = "https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz"
SOURCE_DIR = build/onnxruntime-linux-x64-1.10.0
UNPACK_CMD = tar xvzf
MD5SUM_DIR = $(CURDIR)/md5sum
MD5SUM_FILE = $(MD5SUM_DIR)/$(FILENAME).md5sum
SCRIPT_DIR = $( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
include $(shell rospack find mk)/download_unpack_build.mk
INSTALL_DIR =	'./'


installed.voicevox: $(SOURCE_DIR)/unpacked
	mkdir -p $(INSTALL_DIR)/lib
	cp build/onnxruntime-linux-x64-1.10.0/lib/* $(INSTALL_DIR)/lib

clean:
	rm -rf $(TARBALL)
	rm -rf $(SOURCE_DIR)
	rm -rf $(INSTALL_DIR)/bin
	rm -rf $(INSTALL_DIR)/include
	rm -rf $(INSTALL_DIR)/lib
	rm -rf $(INSTALL_DIR)/man
	rm -rf $(INSTALL_DIR)/share
	rm -rf installed.voicevox
	rm -rf build
	rm -rf patch
