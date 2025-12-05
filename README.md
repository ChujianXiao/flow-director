Flow Director
==========

This repository contains a set of MATLAB scripts that use optical flow to alleviate motion sickness when viewing equirectangular 360 degree videos.

**Authors: ChujianXiao(mattcxiao@gmail.com), SemberStratos**

# Introduction

This repository contains multiple scripts for processing equirectangular videos to provide a better viewing experience. 
1. creatDimMap.m - This script converts equirectangular video frames to cube maps, performs Farneback's optical flow algorithm, then decreases the brightness of pixels based on their optical flow vector magnitude. This reduces the effect of high-frequency objects, which are a major culprit of motion sickness. 
2. foeVisualGuidance.m - This script performs the same optical flow procedure as in 1, but estimates the direction of motion based on the Focus of Expansion and Contraction. It then draws a ring on the output video based on the estimated direction of motion to guide the viewers visually, reducing motion sickness.

# Acknowledgements

This project uses code from the following sources:

- https://github.com/rayryeng/equi2cubic - A MATLAB script to convert equirectangular images (2:1 aspect ratio) to cube maps 
- https://github.com/rayryeng/cubic2equi - The reverse script that converts cube maps to equirectangular images

# Dependencies

The code was only tested on the newest version of MATLAB with the Image Processing Toolbox and Machine Vision Toolbox installed.
foeVisualGuidance.m also requires the Navigation Toolbox, as the estimated motion directions are interpolated with this toolbox's [slerp()](https://www.mathworks.com/help/nav/ref/quaternion.slerp.html) function.

# How to run the code

1. Label your equirectangular input video as "input.mp4".
2. Run creatDimMap.m or foeVisualGuidance.m depending on the effect you want to apply.

# Assumptions

No error checking is involved with this code. Input videos are assumed to be in equirectangular projection, and have a 2:1 aspect ratio.

# License
This code is protected under the MIT License. Feel free to use the code in any way or modify it, but please reference where the source came from. Thank you for viewing or using our project!
