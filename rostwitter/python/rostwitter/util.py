import os
import unicodedata
import yaml

import rospy


def load_oauth_settings(yaml_path):
    if not os.path.exists(yaml_path):
        rospy.logerr('"{}" not found'.format(yaml_path))
        rospy.logerr("$ get access token from https://apps.twitter.com/")
        rospy.logerr("cat {} <<EOF".format(yaml_path))
        rospy.logerr("CKEY: xxx")
        rospy.logerr("CSECRET: xxx")
        rospy.logerr("AKEY: xxx")
        rospy.logerr("ASECRET: xxx")
        rospy.logerr("EOF")
        return None, None, None, None
    with open(yaml_path, 'r') as f:
        key = yaml.load(f)
        ckey = key['CKEY']
        csecret = key['CSECRET']
        akey = key['AKEY']
        asecret = key['ASECRET']
    return ckey, csecret, akey, asecret


def count_tweet_text(text):
    count = 0
    for c in text:
        if unicodedata.east_asian_width(c) in 'FWA':
            count += 2
        else:
            count += 1
    return count


def split_tweet_text(text, length=280):
    texts = []
    split_text = ''
    count = 0
    for c in text:
        if count == 281:
            # last word is zenkaku.
            texts.append(split_text[:-1])
            split_text = split_text[-1:]
            count = 2
        elif count == 280:
            texts.append(split_text)
            split_text = ''
            count = 0
        split_text += c
        if unicodedata.east_asian_width(c) in 'FWA':
            count += 2
        else:
            count += 1
    if count != 0:
        texts.append(split_text)
    return texts
