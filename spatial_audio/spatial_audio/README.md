# spatial_audio

This package provides the spatial_audio server node. The node will play spatial_audio from virtual spatial audio sources generated from requests.

A request usually contains id, action type, frame_id, pose and topicname of stream audio data and meta information. Please see sample client script in spatial_audio_demo package for details.

## Services

### '~play_source' ( spatial_audio_msgs/PlaySpatialAudio )

a service server for virtual audio source management.

## Parameters

### '~num_spinthread' ( int, default: 5 )

number of threads for service and subscriber callbacks

### '~max_duration' ( float, default: 10.0 )

Duration to store tf data in tf buffer.

### '~head_frame_id' ( string, default: "head_link" )

frame_id for the center of a virtual head.

## Subscriber for each audio source

### request.stream_topic_audio ( string )

rostopic name for audio stream data ( audio_common_msgs/AudioData )

### request.stream_topic_info ( string )

rostopic name for audio stream meta information ( audio_common_msgs/AudioInfo )
