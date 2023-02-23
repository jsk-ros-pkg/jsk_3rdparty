# zdepth\_image\_transport

ROS zdepth image transport plugin

`zdepth_image_transport` is for faster and lighter depth compression.

[Zdepth](https://github.com/catid/Zdepth) is a depth compression method with zstd library.

## Supported format

Currently supported formats are as below:

- [x] `16UC1`
- [x] `32FC1`

## How to try

```
roslaunch openni2 openni2.launch
rosrun image_transport republish zdepth in:=/camera/depth/image_raw raw out:=/camera/depth_decompressed/image_raw raw
```

## Visual Comparison

### ZDepth (This package)

With zdepth\_image\_transport, we can get 30Hz rgb and 30Hz Zdepth.

Left: Original, Right: compressed -> decompressed

https://user-images.githubusercontent.com/9300063/199556928-821fe403-f63a-456a-a633-a407bf5fe384.mp4

### CompressedDepth (png)

Because compression of png is so slow, it causes slow down for other depth topics.
(image\_transport is serial process, and if one compression is slow, others will be slow down).
The compression is 4Hz, so the raw depth is also 4Hz.
and the decompression is 4Hz.
Ofcourse, we can make it faster with lower png\_level, but the compression will be worse.

Left: Original, Right: compressed -> decompressed

https://user-images.githubusercontent.com/9300063/199556980-136d74a3-0a09-4df1-9e8e-354b7b65463f.mp4

## Frequency Comparison

### Compression

- ZDepth

```
$ rostopic hz /camera/depth/image_raw/zdepth
subscribed to [/camera/depth/image_raw/zdepth]
average rate: 29.973
        min: 0.033s max: 0.034s std dev: 0.00010s window: 17
```

- Compressed Depth (png)

```
$ rostopic hz /camera/depth/image_raw/compressedDepth
subscribed to [/camera/depth/image_raw/compressedDepth]
average rate: 4.155
        min: 0.241s max: 0.241s std dev: 0.00000s window: 2
```

### Decompression

- Zdepth

```
$ rostopic hz /camera/depth/image_raw/zdepth
subscribed to [/camera/depth/image_raw/zdepth]
average rate: 29.609
        min: 0.013s max: 0.044s std dev: 0.00686s window: 29

```

- Compressed Depth (png)

```
$ rostopic hz /camera/depth_decompressed/image_raw
subscribed to [/camera/depth_decompressed/image_raw]
average rate: 4.175
        min: 0.233s max: 0.244s std dev: 0.00462s window: 4
```

## Topic size Comparison

- Original: `<array type: uint8, length: 614400>`

- ZDepth: `<array type: uint8, length: 35006>`
- compressedDepth (png): `<array type: uint8, length: 61084>`
