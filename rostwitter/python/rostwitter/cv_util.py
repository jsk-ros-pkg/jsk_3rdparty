import base64
import imghdr
import os.path
import re

import cv2
import numpy as np
import rospy


base64_and_filepath_image_pattern = re.compile(r'((?:/9j/)(?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)? ?|/\S+\.(?:jpeg|jpg|png|gif))')


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


def get_image_from_text(text):
    if base64_and_filepath_image_pattern.match(text) is None:
        return None

    if os.path.exists(text):
        path = text
        if imghdr.what(path) in ['jpeg', 'jpg', 'png', 'gif']:
            with open(path, 'rb') as f:
                return f.read()
    else:
        succ = is_base64_image(text)
        if succ:
            bin = text.split(",")[-1]
            bin = base64.b64decode(bin)
            bin = np.frombuffer(bin, np.uint8)
            return bin


def extract_media_from_text(text):
    texts = base64_and_filepath_image_pattern.split(text)
    target_texts = list(filter(lambda x: x is not None and len(x.strip()) > 0, texts))

    split_texts = ['']
    imgs_list = []

    texts = []
    imgs = []
    for text in target_texts:
        img = get_image_from_text(text)
        if img is None:
            split_texts.append(text)
            imgs_list.append(imgs)
            imgs = []
        else:
            imgs.append(img)

    if len(imgs) > 0:
        imgs_list.append(imgs)
    if len(split_texts) > 0:
        if len(split_texts[0]) == 0 and len(imgs_list[0]) == 0:
            split_texts = split_texts[1:]
            imgs_list = imgs_list[1:]
    return imgs_list, split_texts
