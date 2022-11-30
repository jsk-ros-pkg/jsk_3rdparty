import nfc


class NFCInterface:

    def __init__(self,
                 path = 'usb',
                 targets = ['106A', '106B', '212F']):
        """
        Args:
          path(str):
            path to NFC device.
            please see reference of nfc.ContactlessFrontEnd.open().
            https://nfcpy.readthedocs.io/en/latest/modules/clf.html#nfc.clf.ContactlessFrontend.open
        """
        self.targets = targets
        self.clf = nfc.ContactlessFrontend(path)

    def __del__(self):

        self.clf.close()

    def read_card(self, targets=None):

        if targets is None:
            targets = self.targets
        tag = self.clf.connect(
            rdwr={
                'targets': targets,
                'on-connect': lambda tag: False
            }
        )
        return tag
