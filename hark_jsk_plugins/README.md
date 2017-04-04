hark_jsk_plugins
================

JSK Plugins for HARK-ROS

### Installation

Follow the [official site](http://www.hark.jp/wiki.cgi?page=HARK+Installation+Instructions) for instruction.

1. Setup HARK

  ```bash
sudo bash -c 'echo -e "deb http://archive.hark.jp/harkrepos $(lsb_release -cs) non-free\ndeb-src http://archive.hark.jp/harkrepos $(lsb_release -cs) non-free" > /etc/apt/sources.list.d/hark.list'
wget -q -O - http://archive.hark.jp/harkrepos/public.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install autoconf harkfd harkfd-dev libharkio2-dev hark-ros-stacks-indigo hark-ros-indigo
```

2. Compile Plugins


  ```bash
./autogen.sh
./configure --prefix=/usr --with-hark-inc=/usr/include/hark --enable-ros --with-harkio2-inc=/usr/include/harkio2 --with-harkio2-lib=/usr/include/harkio2/ --enable-harkio2
make
sudo make install
```

### Supported Platform

ROS cturtle / diamondback / electric / groovy / hydro / indigo

### Author

Shohei Fujii <<s-fujii@jsk.imi.i.u-tokyo.ac.jp>>
