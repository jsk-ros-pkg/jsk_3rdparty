# nfc_ros

This package is for nfc communication.

Currently only tag reading is supported.

## How to use

First build this package

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd src
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git
catkin build nfc_ros
```

After that, connect NFC device and please activate virtualenv generated from nfc_ros and run nfcpy

```bash
source ~/catkin_ws/devel/share/nfc_ros/venv/bin/activate
python -m nfc
```

This will show a message like this if you do not configure udev for NFC device.

```bash
~/ros/ws_jsk_ros_pkg/devel/share/nfc_ros/venv $ python -m nfc
This is the 1.0.4 version of nfcpy run in Python 3.6.9
on Linux-5.4.0-136-generic-x86_64-with-Ubuntu-18.04-bionic
I'm now searching your system for contactless devices
** found usb:054c:06c3 at usb:005:095 but access is denied
-- the device is owned by 'root' but you are 'sktometometo'
-- also members of the 'root' group would be permitted
-- you could use 'sudo' but this is not recommended
-- better assign the device to the 'plugdev' group
   sudo sh -c 'echo SUBSYSTEM==\"usb\", ACTION==\"add\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"06c3\", GROUP=\"plugdev\" >> /etc/udev/rules.d/nfcdev.rules'
   sudo udevadm control -R # then re-attach device
I'm not trying serial devices because you haven't told me
-- add the option '--search-tty' to have me looking
-- but beware that this may break other serial devs
Sorry, but I couldn't find any contactless device
```

Please follow instructions until you get message like below

```bash
~/ros/ws_jsk_ros_pkg/devel/share/nfc_ros/venv $ python -m nfc
This is the 1.0.4 version of nfcpy run in Python 3.6.9
on Linux-5.4.0-136-generic-x86_64-with-Ubuntu-18.04-bionic
I'm now searching your system for contactless devices
** found SONY RC-S380/P NFC Port-100 v1.11 at usb:005:097
I'm not trying serial devices because you haven't told me
-- add the option '--search-tty' to have me looking
-- but beware that this may break other serial devs
```

After that, you can now run `demo.launch`.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch nfc_ros demo.launch
```

You can get messages from `/nfc_ros_node/tag` topic when you put NFC tag to NFC reader.

```bash
$ rostopic echo /nfc_ros_node/tag
tag_type: 401
ndef_records: []
identifier: "083554be"
---
tag_type: 401
ndef_records: []
identifier: "08aa99aa"
---
tag_type: 401
ndef_records: []
identifier: "08f1b023"
---
```

## Tag editing

Currently nfc_ros does not support tag writing. If you want to customize your NFC Tag, please use other tools (e.g. NFC Tools [for Android](https://play.google.com/store/apps/details?id=com.wakdev.wdnfc&hl=ja&gl=US&pli=1) and [for iPhone](https://apps.apple.com/jp/app/nfc-tools/id1252962749). )

## Support devices

### NFC Interface

Basically we can use any NFC devices supported by [nfcpy](https://nfcpy.readthedocs.io/en/latest/index.html). Currently devices below are test.

- [RC-S300](https://www.sony.co.jp/Products/felica/consumer/products/RC-S300.html)
- [RC-S300/P]()

### NFC Tags/Cards

Available tags/cards types depend on which type of NFC Interface you use. If you use RC-S300 you can use almost any type of tags. (Type2 tags like Ultralight, NXP Mifare, Ultralight C and Type3 tags like Sony FeliCa tag and so on)

For examples.

- [SANWA SUPPLY MM-NFCT](https://www.sanwa.co.jp/product/syohin?code=MM-NFCT#spec-area) (Tagtype: ISO14443 type-A)
