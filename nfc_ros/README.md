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

This will show messages and if you need to do something, there will be an instruction. please follow that.

After that, please build this package. Then you can run `demo.launch`.

```bash
roslaunch nfc_ros demo.launch
```

Currently nfc_ros does not support tag writing. If you want to customize your NFC Tag, please use other tools (e.g. NFC Tools [for Android](https://play.google.com/store/apps/details?id=com.wakdev.wdnfc&hl=ja&gl=US&pli=1) and [for iPhone](https://apps.apple.com/jp/app/nfc-tools/id1252962749). )

## Support devices

### NFC Interface

Basically we can use any NFC devices supported by [nfcpy](https://nfcpy.readthedocs.io/en/latest/index.html). Currently devices below are test.

- [RC-S300](https://www.sony.co.jp/Products/felica/consumer/products/RC-S300.html)

### NFC Tags/Cards

Available tags/cards types depend on which type of NFC Interface you use. If you use RC-S300 you can use almost any type of tags.
