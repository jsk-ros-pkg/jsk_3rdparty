import os
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
