all: openface 

INSTALL_DIR=`rospack find libopenface`
FILENAME =  OpenFace-a7b819f57a7cf12dbe7599644941c54f5e1a1a2b.zip
TARBALL = build/$(FILENAME)
SOURCE_DIR=build/OpenFace-a7b819f57a7cf12dbe7599644941c54f5e1a1a2b
TARBALL_URL =  https://github.com/knorth55/OpenFace/archive/a7b819f57a7cf12dbe7599644941c54f5e1a1a2b.zip
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
