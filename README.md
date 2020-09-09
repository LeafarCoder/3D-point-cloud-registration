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

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Fig_1.PNG)

*Figure 1: Acquisition tool and RGB/depth pair examples*

## 1.2 Pin-hole camera model
The pin-hole camera model explains how a camera, as a mathematical function abstraction, works. This model describes a camera as a transformation that projects 3D world points onto 2D points on the image plane.

To elaborate on the model let us first define 
<img src="https://render.githubusercontent.com/render/math?math=X=[X,Y,Z]^T">
as the coordinates of some 3D world point and
<img src="https://render.githubusercontent.com/render/math?math=x=[x,y]^T">
as the coordinates of that same point when projected onto the image plane.

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Fig_2.PNG)

*Figure 2: Pin-hole camera model*

To project X onto the image plane we first need another point in the 3D world called the optical center, O.
If we define the focal distance, f, as the closest distance from the optical center and the image plane then the
projected points are given by Equation 1.

<img src="https://render.githubusercontent.com/render/math?math=y=f\frac{X}{Z} ,  x=f\frac{Y}{Z}    (1)">


If the focal distance is considered to be unitary (f=1) and homogeneous coordinates are used instead, then the following system is obtained:

<img src="https://render.githubusercontent.com/render/math?math=\lambda \begin{bmatrix}x\\ y\\ z\end{bmatrix}=\begin{bmatrix}1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & \end{bmatrix}\begin{bmatrix}X\\ Y\\ Z\\ 1\end{bmatrix}">

The optical axis does not always intersect the image plane at the origin, as there is an offset (vertical and/or horizontal), which can be described with a combination of a rotation, R and a translation, T. There is also a change of units: from the 3D world in meters (or some multiple) to the image plane in pixels. There is a conversion factor to account for these changes. Together, these values are called intrinsic parameters as they define properties of the image formation affected by the camera itself, and are represented by a matrix, K.

<img src="https://render.githubusercontent.com/render/math?math=\lambda \begin{bmatrix}u\\ v\\ 1\end{bmatrix}=K\begin{bmatrix}1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & \end{bmatrix}\begin{bmatrix}R & T\\ 0^T & 1\end{bmatrix}\begin{bmatrix}X\\ Y\\ Z\\ 1\end{bmatrix}">

Since we are now in image coordinates in pixels, these are represented by u and v instead of x and y, so
they are distinguishable.

## 1.3 Feature detection with SIFT
The Shift Invariant Feature Transform (SIFT) is an algorithm for feature detection and matching.

The SIFT algorithm builds the scale space of the image. To do so, it filters the image with Gaussian filters of different standard deviations. The set of images where each has been filtered with a different standard deviation Gaussian is called an octave. After the first octave is built, the image is downsampled to build another octave, using the same procedure, at a different scale.

Points are considered ”good” as features when they are found in a region that changes locally in many directions - keypoints. To find these points, one can consider the extrema of the laplacian of the image, as these match the criteria to be considered ”good” points/features. For each octave, these extrema can be well approximated by computing the difference of consecutive Gaussian-filtered images, and taking the extrema of that resulting image. Local gradients of the regions around keypoints are then computed, with the average direction of the gradient being called the reference orientation for that keypoint. Having found the keypoints and their reference orientation, the SIFT algorithm builds histograms of the gradient according to that reference orientation, creating a numerical description of the region where the keypoint is found - called a descriptor. Because it is computed on a reference orientation, this makes SIFT robust against rotation. And because the
descriptor is stored according to the key point position it is invariant to translations as well. The histograms are normalized in a posterior step, making the descriptor invariant to global, uniform illumination changes. Keypoints are extracted at different scales and blur levels and all subsequent computations are performed within the scale space framework, making their descriptors scale invariant too.

Features are matched using their descriptors, which are invariant to many image transformations, thus providing a set of points which can be used to match point clouds.

## 1.4 Model fitting to noisy data with RANSAC
Random Sample Consensus (RANSAC) is a method that iteratively searches for the best estimate for the parameters of a model. It works on the assumption that a given dataset might be composed of both inliers which are explained by the model (and may contain noise) and outliers. A clear distinction between noise and outliers is important. While noise is a perturbance of the signal that is explained by some distribution (most commonly a normal distribution), an outlier is a data point that deviates so much from the model that the probability of the noise distribution explaining it is very low.

As an input, the algorithm requires the model, a set of N points, the minimum number of points to instantiate a particular model (n), the probability of finding an inlier on the dataset (p), and a threshold value to know if a point is explained by the model (d).

The procedure of the algorithm follows the following steps.

1. Randomly sample n points and instantiate the model from where we obtain the model fitted parameters, <img src="https://render.githubusercontent.com/render/math?math=\widehat{\Theta}">.
2. For all points compute the error <img src="https://render.githubusercontent.com/render/math?math=\varepsilon _i = |Y_i-\widehat{\Theta }X_i|">
3. Classify every point as Inlier or Outlier based on the rule:

4. Count number of Inliers. If this is the largest number of inliers so far then update maximum and keep these inliers as bestInliers.
5. Repeat steps from 1. to 4. and stop after k iterations.

Since noise is randomly distributed, it is highly unlikely that the chosen model will fit to points that are not
inliers. With the gathered inlier points, one can estimate the transformation given the model.
Since the probability of success for finding a correct random sample of n points from where to instantiate
the model is given by 
<img src="https://render.githubusercontent.com/render/math?math=P=1-(1-p^n)^k">
, then, to estimate the number of iterations to use, we can use:
