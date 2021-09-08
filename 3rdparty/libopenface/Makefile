all: openface 

INSTALL_DIR=`rospack find libopenface`
FILENAME =  OpenFace-1d043c37a37f7b125cb3e0d854c8eee7df2d736b.zip
TARBALL = build/$(FILENAME)
SOURCE_DIR= build/OpenFace-1d043c37a37f7b125cb3e0d854c8eee7df2d736b
TARBALL_URL =  https://github.com/knorth55/OpenFace/archive/1d043c37a37f7b125cb3e0d854c8eee7df2d736b.zip
UNPACK_CMD = unzip -q
MD5SUM_DIR = $(CURDIR)
MD5SUM_FILE = $(MD5SUM_DIR)/$(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

openface:$(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && mkdir -p build && cd build && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DCMAKE_BUILD_TYPE=Release .. && make install

clean:
	rm -rf $(SOURCE_DIR)/build

wiped: Makefile	make wipe
	touch wiped

wipe: 	clean
	touch wiped
