# Emotion Analyzer Service using Hume API (ROS1)

This ROS1 package provides a service to analyze emotions from a given text using the [Hume AI](https://www.hume.ai/) API.

## Requirements

- ROS1 Noetic
- Python 3.8+
- `hume` Python package
- An API key from Hume AI

## Installation

Install the required Python package:

```bash
pip install hume
```

## Usage

### Set your Hume API key
Before running the node, set your API key using rosparam:
```bash
rosparam set /hume_api_key "your_hume_api_key_here"
```
Note that you need "".

### Run the ROS node
```bash
rosrun emotion_analyzer analyze_text_service.py
```

### Call the service
```bash
rosservice call /analyze_text "text: 'text you want to analyze'"
```

