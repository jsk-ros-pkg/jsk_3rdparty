import base64
import imghdr
import os.path
import re

import cv2
import numpy as np
import rospy


base64_and_filepath_image_pattern = re.compile(r'((/9j/)(?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)?|/\S+\.(jpeg|jpg|png|gif))')


def encode_image_cv2(img, quality=90):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    result, encimg = cv2.imencode('.jpg', img, encode_param)
    b64encoded = base64.b64encode(encimg).decode('ascii')
    return b64encoded


def decode_image_cv2(b64encoded):
    bin = b64encoded.split(",")[-1]
    bin = base64.b64decode(bin)
    bin = np.frombuffer(bin, np.uint8)
    img = cv2.imdecode(bin, cv2.IMREAD_COLOR)
    return img


def is_base64_image(b64encoded):
    try:
        decode_image_cv2(b64encoded)
    except Exception as e:
        rospy.logerr(str(e))
        return False
    return True


def extract_media_from_text(text):
    imgs = []
    for m in base64_and_filepath_image_pattern.finditer(text):
        if os.path.exists(m.group()):
            path = m.group()
            if imghdr.what(path) in ['jpeg', 'png', 'gif']:
                with open(path, 'rb') as f:
                    imgs.append(f.read())
        else:
            succ = is_base64_image(m.group())
            if succ:
                b64encoded = m.group()
                bin = b64encoded.split(",")[-1]
                bin = base64.b64decode(bin)
                bin = np.frombuffer(bin, np.uint8)
                imgs.append(bin)
    # replace all base64 texts to empty string.
    text = base64_and_filepath_image_pattern.sub('', text)
    return imgs, text
