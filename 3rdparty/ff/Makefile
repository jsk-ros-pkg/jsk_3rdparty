all: installed
# ff
FILENAME = FF-v2.3.tgz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://github.com/jsk-ros-pkg/archives/raw/master/$(FILENAME)
SOURCE_DIR = build/FF-v2.3
UNPACK_CMD = tar xzf
MD5SUM_DIR = $(CURDIR)
MD5SUM_FILE = $(MD5SUM_DIR)/$(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

PATCH = ff.patch # currently not used
INSTALL_DIR = `rospack find ff`
MAKE = make
MAKE_ARGS = ADDONS=-DYY_SKIP_YYWRAP

installed:$(SOURCE_DIR)/unpacked
	rm -f $(SOURCE_DIR)/ff
	(cd $(SOURCE_DIR) && $(MAKE) clean && $(MAKE) CFLAGS='-O6 -g -ansi -Wno-array-bounds -Wno-endif-labels -Wno-format-overflow -Wno-implicit-function-declaration -Wno-int-in-bool-context -Wno-unused-function -Wno-unused-variable -fno-builtin-strncpy -fno-builtin-strcpy  -fno-builtin-strlen -fno-builtin-strcat -fno-builtin-memset -fcommon ' $(MAKE_ARGS))
	mkdir -p bin
	cp $(SOURCE_DIR)/ff bin/
	touch installed

clean:
	rm -rf build
	rm -rf bin
	rm -rf installed

wiped: Makefile
	make wipe
	touch wiped

wipe: 	clean
	touch wiped
