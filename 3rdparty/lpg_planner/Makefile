all: installed

INSTALL_DIR=`rospack find lpg_planner`
FILENAME = lpg1.2-linux.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://github.com/jsk-ros-pkg/archives/raw/master/$(FILENAME)
SOURCE_DIR = build/LPG-1.2-linux
UNPACK_CMD = tar xzf
MD5SUM_DIR = $(CURDIR)
MD5SUM_FILE = $(MD5SUM_DIR)/$(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

installed:$(SOURCE_DIR)/unpacked
	mkdir -p bin
	cp $(SOURCE_DIR)/lpg-1.2 bin/
	touch installed

clean:
	rm -rf bin
	rm -rf installed

wiped: Makefile	make wipe
	touch wiped

wipe: 	clean
	touch wiped
