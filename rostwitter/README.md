# rostwitter

This package is a ROS wrapper for Twitter. You can tweet via ROS.

# How to use

## Get access key for API.

Please get access to the Twitter API. Please refer to the following URL.

https://developer.twitter.com/en/docs/twitter-api/getting-started/getting-access-to-the-twitter-api

After that, save the yaml file in the following format.

```
CKEY: <Your Consumer API Key>
CSECRET: <Your Consumer SECRET API Key>
AKEY: <Your API Key>
ASECRET: <Your API Secret Key>
```

## Launch tweet node

```
roslaunch rostwitter tweet.launch account_info:=<PATH TO YOUR YAML FILE>
```

## Tweet text

You can tweet by simply publish on the `/tweet` topic.

```
rostopic pub /tweet std_msgs/String "Hello. Tweet via rostwitter (https://github.com/jsk-ros-pkg/jsk_3rdparty)"
```

![](./doc/tweet-string.jpg)

If the string to be tweeted exceeds 140 full-width characters or 280 half-width characters, it will be tweeted in the "thread" display.

```
rostopic pub /tweet std_msgs/String """The Zen of Python, by Tim Peters

Beautiful is better than ugly.
Explicit is better than implicit.
Simple is better than complex.
Complex is better than complicated.
Flat is better than nested.
Sparse is better than dense.
Readability counts.
Special cases aren't special enough to break the rules.
Although practicality beats purity.
Errors should never pass silently.
Unless explicitly silenced.
In the face of ambiguity, refuse the temptation to guess.
There should be one-- and preferably only one --obvious way to do it.
Although that way may not be obvious at first unless you're Dutch.
Now is better than never.
Although never is often better than *right* now.
If the implementation is hard to explain, it's a bad idea.
If the implementation is easy to explain, it may be a good idea.
Namespaces are one honking great idea -- let's do more of those!
"""
```

![](./doc/tweet-string-thread.jpg)

## Tweet text with image

You can also tweet along with your images.

If a base64 or image path is inserted in the text, it will jump to the next reply in that section.

### Image path

```
wget https://github.com/k-okada.png -O /tmp/k-okada.png
rostopic pub /tweet std_msgs/String "/tmp/k-okada.png"
```

![](./doc/tweet-image-path.jpg)

### Base64

You can even tweet the image by encoding in base64. The following example is in python.

```python
import rospy
import cv2
import std_msgs.msg
import numpy as np
import matplotlib.cm

from rostwitter.cv_util import extract_media_from_text
from rostwitter.cv_util import encode_image_cv2

rospy.init_node('rostwitter_sample')
pub = rospy.Publisher('/tweet', std_msgs.msg.String, queue_size=1)
rospy.sleep(3.0)

colormap = matplotlib.cm.get_cmap('hsv')

text = 'Tweet with images. (https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/375)\n'
N = 12
for i in range(N):
    text += str(i)
    color = colormap(1.0 * i / N)[:3]
    img = color * np.ones((10, 10, 3), dtype=np.uint8) * 255
    img = np.array(img, dtype=np.uint8)
    text += encode_image_cv2(img)
pub.publish(text)
```

[The result of the tweet.](https://twitter.com/pr2jsk/status/1561995909524705280)
