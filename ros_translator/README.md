# ros_translator

A ROS package for language translation.
This package uses Python package [deep-translator](https://pypi.org/project/deep-translator) as a backend.

## Demo

```bash
roslaunch ros_translator demo
```

And then, please input any text to `xterm` prompt. You will get a translated text.

```bash
Input any text >いづれの御時にか、女御、更衣あまた候ひ給ひける中に、いとやむごとなき際にはあらぬが、すぐれて時めき給ふありけり。
```

```bash
[INFO] [WallTime: 1682560250.555090] [node:/demo_output_node] [func:DEMO.callback]: Translated text: On some occasion, while the nyogo is wearing a lot of clothes, it's not an unavoidable moment, but the excitement is excellent.
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
