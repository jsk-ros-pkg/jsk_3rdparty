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

```bash
roslaunch emotion_analyzer emotion_analyzer.launch api_key:=<your_api_key>
```

### Call the service
```bash
rosservice call /analyze_text "text: '<text you want to analyze>'"
```

