# Encoding ROS2 Image Msg

ffmpeg encode 


### Use ffmpeg to encode images to video
```
ffmpeg -framerate 10 -pattern_type glob -i "SAMPLE/*.jpg" -hwaccel cuda -c:v h264_nvenc -r 10 test_nvenc.mp4 -y

ffmpeg -hwaccel cuda -framerate 10 -pattern_type glob -i "SAMPLE/*.jpg" -c:v h264_nvenc -r 10 test_nvenc.mp4 -y
```

## Round trip test
A script for validating ffmpeg encoding and encoding workflow, simulating the use case for ROS2 messages. It does the following:

- Encode Images to packets that can be store in independent data structures.
- Decode packets back to images.
- Evaluate the compression loss.


