all: libsiftfast

SOURCE_DIR=`rospack find libsiftfast`
INSTALL_DIR=`rospack find libsiftfast`
SVN_DIR = libsiftfast_svn
SVN_URL = http://svn.code.sf.net/p/libsift/code/trunk
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: clone patch libsiftfast clean wipe

build: SVN_UP libsiftfast

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --lib_dirs)

BUILDDIR=$(shell if [ $(DEBUG) ]; then echo builddebug; else echo build; fi)
BUILDDIR=$(shell if [ $(DEBUG) ]; then echo builddebug; else echo build; fi)
CMAKE_BUILD_TYPE=$(shell if [ $(DEBUG) ]; then echo Debug; else echo RelWithDebInfo; fi)

clone: $(SVN_DIR)
	cd $(SVN_DIR) && svn up && mkdir -p $(BUILDDIR)

patch: clone
	cd $(SVN_DIR) && svn revert --recursive . && patch -p0 -f -E < $(SOURCE_DIR)/patches/01.fix_python_binding.patch

libsiftfast: patch
	cd $(SVN_DIR)/$(BUILDDIR) && BOOST_INCLUDEDIR=$(BOOST_INCLUDE_DIRS) BOOST_LIBRARYDIR=$(BOOST_LIBRARY_DIRS) cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) .. && make $(ROS_PARALLEL_JOBS) install

clean:
	cd $(INSTALL_DIR) && make -C $(SVN_DIR) clean

wipe: clean
	cd $(INSTALL_DIR) && rm -rf $(SVN_DIR) bin include lib share patched

