# nfc_ros

This package is for nfc communication.

## How to use

Before use this, please install nfcpy and setup udev for your device.

```bash
pip3 install nfcpy
```

And then, connect your NFCReader to PC and run

```bash
python3 -m nfc
```

This will show messages and if you need to do somethin, there will be an instruction. please follow that.

After that, please build this package and run.

```bash
roslaunch nfc_ros demo.launch
```

If you want to customize your NFC Tag, please use other tools (e.g. NFC Tools [for Android](https://play.google.com/store/apps/details?id=com.wakdev.wdnfc&hl=ja&gl=US&pli=1) and [for iPhone](https://apps.apple.com/jp/app/nfc-tools/id1252962749). )
