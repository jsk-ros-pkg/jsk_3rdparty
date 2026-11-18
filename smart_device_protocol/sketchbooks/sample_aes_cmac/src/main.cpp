// AES_CMAC library example
// by Industrial Shields

#include <AES_CMAC.h>

// Required AES.h from arduino Crypto library https://www.arduino.cc/reference/en/libraries/crypto/
#include <AES.h>

// Key and data from https://tools.ietf.org/html/rfc4493#section-4
const uint8_t key[16] = {
    0x2b,
    0x7e,
    0x15,
    0x16,
    0x28,
    0xae,
    0xd2,
    0xa6,
    0xab,
    0xf7,
    0x15,
    0x88,
    0x09,
    0xcf,
    0x4f,
    0x3c,
};
const uint8_t data_raw[] = {
    0x6b,
    0xc1,
    0xbe,
    0xe2,
    0x2e,
    0x40,
    0x9f,
    0x96,
    0xe9,
    0x3d,
    0x7e,
    0x11,
    0x73,
    0x93,
    0x17,
    0x2a,
    0xae,
    0x2d,
    0x8a,
    0x57,
    0x1e,
    0x03,
    0xac,
    0x9c,
    0x9e,
    0xb7,
    0x6f,
    0xac,
    0x45,
    0xaf,
    0x8e,
    0x51,
    0x30,
    0xc8,
    0x1c,
    0x46,
    0xa3,
    0x5c,
    0xe4,
    0x11,
};

// The output
uint8_t mac[16];

AESTiny128 aes128;
AES_CMAC cmac(aes128);

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    USBSerial.begin(115200);
    delay(1000);
    USBSerial.println("AES_CMAC example");

    // Generate the MAC from data_raw, using the key. The result is stored into mac
    cmac.generateMAC(mac, key, data_raw, sizeof(data_raw));

    // Print the result
    USBSerial.print("MAC: ");
    for (int i = 0; i < sizeof(mac); ++i)
    {
        if (mac[i] < 0x10)
        {
            USBSerial.print('0');
        }
        USBSerial.print(mac[i], HEX);
        USBSerial.print(' ');
    }
    USBSerial.println();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    delay(100);
    USBSerial.print("MAC: ");
    for (int i = 0; i < sizeof(mac); ++i)
    {
        if (mac[i] < 0x10)
        {
            USBSerial.print('0');
        }
        USBSerial.print(mac[i], HEX);
        USBSerial.print(' ');
    }
    USBSerial.println();
}