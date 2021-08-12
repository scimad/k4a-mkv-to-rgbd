# K4A MKV to RGBD converter

Converts a RGBD MKV video (recorded by Azure Kinect) to RGB and Depth image frames

### To build:
```
git clone <[SSH: git@github.com:scimad/k4a-mkv-to-rgbd.git or HTTPS: https://github.com/scimad/k4a-mkv-to-rgbd.git]>
cd k4a-mkv-to-rgbd
mkdir build && cd build
cmake ..
make
```

### Requirements:
* OpenCV
* k4a::k4a
* k4arecord
* TURBOJPEG

### Caveats:
- Might need tweaks for some capturing modes
