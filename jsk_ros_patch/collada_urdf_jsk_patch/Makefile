#
all: urdf_to_collada

GIT_DIR = build/robot_model/src
GIT_URL = git://github.com/ros/robot_model.git
GIT_REVISION = ${SOURCE_DISTRO}

TARBALL     = build/robot_model.zip
TARBALL_URL = https://github.com/ros/${GIT_REPO}/archive/${SOURCE_DISTRO}.zip
SOURCE_DIR  = build/robot_model/src
INITIAL_DIR  = build/${GIT_REPO}-${SOURCE_DISTRO}
MD5SUM_FILE = ${PATCH_DIR}/robot_model.$(shell echo ${SOURCE_DISTRO} | head -c 4).md5sum
UNPACK_CMD  = unzip

GIT_PATCH = ${PATCH_DIR}/use_assimp_devel.patch
ifeq ("${SOURCE_DISTRO}", "923c5d33bd245e82134e8ae02e4c9d379e80eb27") # 1.12.12
GIT_PATCH += ${PATCH_DIR}/fix_issue_18.patch
endif
BUILD_BIN_DIR  = build/robot_model/devel/lib/collada_urdf
#include $(shell rospack find mk)/git_checkout.mk
include $(shell rospack find mk)/download_unpack_build.mk

# disable_ssl:
# 	git config --global http.sslVerify false

catkin_ws:
	mkdir -p build/robot_model/src

$(SOURCE_DIR)/patched:  $(SOURCE_DIR)/unpacked # download_unpack_build use -p0, which did not work with our ${GIT_PATCH}
	$(foreach PATCH, $(GIT_PATCH), patch -d $(SOURCE_DIR) -p1 < $(PATCH) && ) echo patched
	touch patched

urdf_to_collada: catkin_ws $(SOURCE_DIR)/patched
	(cd build/robot_model; catkin init -w .; PKG_CONFIG_PATH=`rospack find assimp_devel`/lib/pkgconfig:${PKG_CONFIG_PATH} catkin build -v -i collada_urdf --force-cmake --no-status --no-notify)
	cp $(BUILD_BIN_DIR)/urdf_to_collada .
	cp $(BUILD_BIN_DIR)/collada_to_urdf .

clean:
	if [ -f build/robot_model/build ] ; then catkin clean -a; fi
	rm -fr build/robot_model/build
	rm -rf installed patched
	rm -f urdf_to_collada

wipe: clean
	rm -rf $(SRC_DIR)
