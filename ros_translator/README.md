# ros_translator

A ROS package for language translation.
This package uses Python package [deep-translator](https://pypi.org/project/deep-translator) as a backend.

## Demo

```bash
roslaunch ros_translator demo
```

And then, please input any text to `xterm` prompt. You will get a translated text.

```bash
Input any text >沙羅双樹の花の色、盛者必衰の理をあらはす
```

```bash
[INFO] [WallTime: 1682560820.715069] [node:/demo_output_node] [func:DEMO.callback]: Translated text: The color of the flowers of the sal tree reveals the reason why prosperity must decline
```

## `ros_translator_node.py` Interface

### Subsriber

* `~input_text` (`std_msgs/String`)

    Input text to be translated.

### Publisher

* `~output_text` (`std_msgs/String`)

    Output text translated.

### Parameters

* `~translator` (`String`, default: `google`)

    Backend translation web service

* `~from_language` (`String`, default: `auto`)

    Input language

* `~to_language` (`String`, default: `en`)

    Target language

* `~api_key` (`String`, default: ``)

    API key for some backend web services
