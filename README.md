# Encoding ROS2 Image Msg

ffmpeg encode 


### Use ffmpeg to encode images to video
```
ffmpeg -framerate 10 -pattern_type glob -i "SAMPLE/*.jpg" -hwaccel cuda -c:v h264_nvenc -r 10 test_nvenc.mp4 -y

ffmpeg -hwaccel cuda -framerate 10 -pattern_type glob -i "SAMPLE/*.jpg" -c:v h264_nvenc -r 10 test_nvenc.mp4 -y
```