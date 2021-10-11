# refer to
# http://blog.sipeed.com/p/677.html
# https://qiita.com/Nabeshin/items/9268dc88927123319549
# https://github.com/sipeed/MaixPy_scripts/blob/master/machine_vision/yolov2_20class.py

import sensor
import time
from machine import UART
from fpioa_manager import fm
from modules import ws2812
import KPU as kpu

# M5StickV Camera Start
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.set_hmirror(1)
sensor.run(1)
sensor.skip_frames()

# RGB LED
class_ws2812 = ws2812(8, 100)

# M5StickV GPIO_UART
fm.register(35, fm.fpioa.UART1_TX, force=True)
fm.register(34, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

# yolo recognition
clock = time.clock()
classes = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']  # NOQA
task = kpu.load(0x500000)
anchor = (1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52)
a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)
recog_flag = True

# Packet header for UART, 4 byte
packet_header = bytearray([0xFF, 0xD8, 0xEA, 0x01])


def RGB_LED(r, g, b):
    class_ws2812.set_led(0, (r, g, b))
    class_ws2812.display()


def send_recog_image(img, obj=None):
    # Create recognition result data. 16+16+4=36 byte
    x_scale = 0.6
    y_scale = 0.6
    if obj is None:
        recog_str = "{:<36}".format("none")
    else:
        label_str = "{:<16}".format(classes[obj.classid()])
        rect = obj.rect()
        rect_str = '{:0=4}{:0=4}{:0=4}{:0=4}'.format(
            int(rect[0] * x_scale), int(rect[1] * y_scale),
            int(rect[2] * x_scale), int(rect[3] * y_scale))
        proba_str = '{:0=4}'.format(int(obj.value() * 100))
        recog_str = label_str + rect_str + proba_str
    # Create recognition image data
    # Crop(Resize) and compress image to reduce data
    img.crop(x_scale=x_scale, y_scale=y_scale)
    img.compress(quality=50)
    recog_flag_byte = bytearray([0x01]) if recog_flag else bytearray([0x00])
    img_size1 = (img.size() & 0xFF0000) >> 16
    img_size2 = (img.size() & 0x00FF00) >> 8
    img_size3 = (img.size() & 0x0000FF) >> 0
    img_size = bytearray([img_size1, img_size2, img_size3])
    padding = bytearray([0x00] * 20)
    # Send data via UART.
    # Header           4 byte
    # recog_flag       1 byte
    # recog result    36 byte
    # image data size  3 byte
    # 0 padding       20 byte
    # total           64 byte
    data_str = packet_header + recog_flag_byte + recog_str + img_size + padding
    uart.write(data_str)
    # Wait for M5Stack to clear UART buffer
    time.sleep(0.1)
    # Send image
    uart.write(img)
    print("img.size(): {}".format(img.size()))


def send_data():
    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)
    if code:
        print('len(code): {}'.format(len(code)))
        send_recog_image(img, code[0])
    else:
        send_recog_image(img)


# Start signal LED
RGB_LED(255, 255, 255)
time.sleep(0.3)
RGB_LED(0, 0, 0)
time.sleep(0.3)
RGB_LED(255, 255, 255)

# Main loop
while(True):
    clock.tick()
    try:
        if recog_flag:
            send_data()
            time.sleep(0.5)  # Save computation power
    except Exception as e:
        print(e)
        print("Error occurred, but continue")
    time.sleep(0.01)
    print(clock.fps())
a = kpu.deinit(task)
