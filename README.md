# 3D-point-cloud-registration

This project was done in the context of the Vision and Image Processing course lectured at Instituto Superior Técnico (Fall 2019)

- [Introduction](#introduction)
- [Introduction](#introduction)
- [Introduction](#introduction)
- [Introduction](#introduction)

## Introduction

The main purpose of this project is to reconstruct a 3D scene from a set of RGB and depth images (in 2D) acquired from a Kinect camera.

The problem is divided into sub-problems and each one was tackled by a sub-solution, an individual cell of the overall solution in order to increase code modularity and testability. An overall view of the proposed solution is shown in the flowchart present in Figure 3.
The following Sections present the acquisition tool (Section 1.1), and some theory (Sections 1.2 to 1.6) that
back up the solutions used (further discussed on Section 2).

## 1.1 Kinect camera
The acquisition of both RGB and depth images was performed using a Kinect camera (Intel Realsense).
This technology has a regular RGB camera to read the RGB image and an infrared projector and camera which measures the distance from the Kinect to the 3D world points that appear on the 2D RGB image.

Some drawbacks come from the fact the Kinect uses infrared (IR) light to perceive depth in a scene. Firstly, it hinders its use on outdoor situations due the intense IR light from the sun that would disrupt the projected IR light from the Kinect. Secondly, objects that do not effectively reflect IR light (mainly black-coloured objects) will cause erroneous readings (saturating at zero). This effect can be observed in Figure 4c where the monitor and computer towers at the back of the room have a zero depth value, meaning no reading was possible.

All images acquired with the Kinect (RGB and depth) have a resolution of 640x480 pixels.

## 1.2 Pin-hole camera model
The pin-hole camera model explains how a camera, as a mathematical function abstraction, works. This model describes a camera as a transformation that projects 3D world points onto 2D points on the image plane.

To elaborate on the model let us first define
<img src="https://render.githubusercontent.com/render/math?math=\pmb{X}=[X, Y, Z]^T">
as the coordinates of some 3D world point and
<img src="https://render.githubusercontent.com/render/math?math=\pmb{x}=[x, y]^T">
as the coordinates of that same point when projected onto the image plane.
