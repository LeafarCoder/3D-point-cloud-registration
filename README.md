# 3D-point-cloud-registration

This project was done in the context of the Vision and Image Processing course lectured at Instituto Superior Técnico (Fall 2019)

- [Introduction](#introduction)
- [1.1 Kinect camera](#11-kinect-camera)
- [1.2 Pin-hole camera model](#12-pin-hole-camera-model)
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

To elaborate on the model let us first define <p align="center"><img src="https://rawgit.com/LeafarCoder/3D-point-cloud-registration/svgs/svgs/ece379bc94ea9c45a8fa9e846c29ebf8.svg?invert_in_darkmode" align=middle width=106.95356594999998pt height=18.7598829pt/></p> as the coordinates of some 3D world point and <p align="center"><img src="https://rawgit.com/LeafarCoder/3D-point-cloud-registration/svgs/svgs/7df2c47253d49b8cb2872523baaaafb9.svg?invert_in_darkmode" align=middle width=80.7169605pt height=18.7598829pt/></p> as the coordinates of that same point when projected onto the image plane.

To project X onto the image plane we first need another point in the 3D world called the optical center, O.
If we define the focal distance, f, as the closest distance from the optical center and the image plane then the
projected points are given by Equation 1.


If the focal distance is considered to be unitary <p align="center"><img src="https://rawgit.com/LeafarCoder/3D-point-cloud-registration/svgs/svgs/f1aeca7a49e452f936aa5128a197e71b.svg?invert_in_darkmode" align=middle width=52.7396859pt height=16.438356pt/></p> and homogeneous coordinates are used instead, then
the following system is obtained:


The optical axis does not always intersect the image plane at the origin, as there is an offset (vertical and/or
horizontal), which can be described with a combination of a rotation, R and a translation, T. There is also
a change of units: from the 3D world in meters (or some multiple) to the image plane in pixels. There is a
conversion factor to account for these changes. Together, these values are called intrinsic parameters as they
define properties of the image formation affected by the camera itself, and are represented by a matrix, K.