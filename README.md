# 3D-point-cloud-registration

This project was done in the context of the Vision and Image Processing course lectured at Instituto Superior Técnico (Fall 2019)

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/cover.png)

---
- [1 Introduction](#1-introduction)
  - [1.1 Kinect camera](#11-kinect-camera)
  - [1.2 Pin-hole camera model](#12-pin-hole-camera-model)
  - [1.3 Feature detection with SIFT](#13-feature-detection-with-sift)
  - [1.4 Model fitting to noisy data with RANSAC](#14-model-fitting-to-noisy-data-with-ransac)
  - [1.5 Estimating transformation between two point clouds](#15-estimating-transformation-between-two-point-clouds)
  - [1.6 The ICP algorithm](#16-the-icp-algorithm)
- [2 Implementation](#2-implementation)

## 1 Introduction

The main purpose of this project is to reconstruct a 3D scene from a set of RGB and depth images (in 2D) acquired from a Kinect camera.

The problem is divided into sub-problems and each one was tackled by a sub-solution, an individual cell of the overall solution in order to increase code modularity and testability. An overall view of the proposed solution is shown in the flowchart present in Figure 3.
The following Sections present the acquisition tool (Section 1.1), and some theory (Sections 1.2 to 1.6) that
back up the solutions used (further discussed on Section 2).

### 1.1 Kinect camera
The acquisition of both RGB and depth images was performed using a Kinect camera (Intel Realsense).
This technology has a regular RGB camera to read the RGB image and an infrared projector and camera which measures the distance from the Kinect to the 3D world points that appear on the 2D RGB image.

Some drawbacks come from the fact the Kinect uses infrared (IR) light to perceive depth in a scene. Firstly, it hinders its use on outdoor situations due the intense IR light from the sun that would disrupt the projected IR light from the Kinect. Secondly, objects that do not effectively reflect IR light (mainly black-coloured objects) will cause erroneous readings (saturating at zero). This effect can be observed in Figure 4c where the monitor and computer towers at the back of the room have a zero depth value, meaning no reading was possible.

All images acquired with the Kinect (RGB and depth) have a resolution of 640x480 pixels.

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Fig_1.PNG)

*Figure 1: Acquisition tool and RGB/depth pair examples*

### 1.2 Pin-hole camera model
The pin-hole camera model explains how a camera, as a mathematical function abstraction, works. This model describes a camera as a transformation that projects 3D world points onto 2D points on the image plane.

To elaborate on the model let us first define 
<img src="https://render.githubusercontent.com/render/math?math=\mathbf{X}=[X,Y,Z]^T">
as the coordinates of some 3D world point and
<img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}=[x,y]^T">
as the coordinates of that same point when projected onto the image plane.

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Fig_2.PNG)

*Figure 2: Pin-hole camera model*

To project X onto the image plane we first need another point in the 3D world called the optical center, O.
If we define the focal distance, f, as the closest distance from the optical center and the image plane then the
projected points are given by Equation 1.

<img src="https://render.githubusercontent.com/render/math?math=y=f\frac{X}{Z} \quad,\quad x=f\frac{Y}{Z}\quad \quad (1)">

If the focal distance is considered to be unitary (f=1) and homogeneous coordinates are used instead, then the following system is obtained:

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Eq_2.PNG)

The optical axis does not always intersect the image plane at the origin, as there is an offset (vertical and/or horizontal), which can be described with a combination of a rotation, R and a translation, T. There is also a change of units: from the 3D world in meters (or some multiple) to the image plane in pixels. There is a conversion factor to account for these changes. Together, these values are called intrinsic parameters as they define properties of the image formation affected by the camera itself, and are represented by a matrix, K.

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Eq_3.PNG)

Since we are now in image coordinates in pixels, these are represented by u and v instead of x and y, so they are distinguishable.

### 1.3 Feature detection with SIFT
The Shift Invariant Feature Transform (SIFT) is an algorithm for feature detection and matching.

The SIFT algorithm builds the scale space of the image. To do so, it filters the image with Gaussian filters of different standard deviations. The set of images where each has been filtered with a different standard deviation Gaussian is called an octave. After the first octave is built, the image is downsampled to build another octave, using the same procedure, at a different scale.

Points are considered ”good” as features when they are found in a region that changes locally in many directions - keypoints. To find these points, one can consider the extrema of the laplacian of the image, as these match the criteria to be considered ”good” points/features. For each octave, these extrema can be well approximated by computing the difference of consecutive Gaussian-filtered images, and taking the extrema of that resulting image. Local gradients of the regions around keypoints are then computed, with the average direction of the gradient being called the reference orientation for that keypoint. Having found the keypoints and their reference orientation, the SIFT algorithm builds histograms of the gradient according to that reference orientation, creating a numerical description of the region where the keypoint is found - called a descriptor. Because it is computed on a reference orientation, this makes SIFT robust against rotation. And because the
descriptor is stored according to the key point position it is invariant to translations as well. The histograms are normalized in a posterior step, making the descriptor invariant to global, uniform illumination changes. Keypoints are extracted at different scales and blur levels and all subsequent computations are performed within the scale space framework, making their descriptors scale invariant too.

Features are matched using their descriptors, which are invariant to many image transformations, thus providing a set of points which can be used to match point clouds.

### 1.4 Model fitting to noisy data with RANSAC
Random Sample Consensus (RANSAC) is a method that iteratively searches for the best estimate for the parameters of a model. It works on the assumption that a given dataset might be composed of both inliers which are explained by the model (and may contain noise) and outliers. A clear distinction between noise and outliers is important. While noise is a perturbance of the signal that is explained by some distribution (most commonly a normal distribution), an outlier is a data point that deviates so much from the model that the probability of the noise distribution explaining it is very low.

As an input, the algorithm requires the model, a set of N points, the minimum number of points to instantiate a particular model (n), the probability of finding an inlier on the dataset (p), and a threshold value to know if a point is explained by the model (d).

The procedure of the algorithm follows the following steps.

1. Randomly sample n points and instantiate the model from where we obtain the model fitted parameters, <img src="https://render.githubusercontent.com/render/math?math=\widehat{\Theta}">.
2. For all points compute the error <img src="https://render.githubusercontent.com/render/math?math=\varepsilon _i = |Y_i-\widehat{\Theta }X_i|">
3. Classify every point as Inlier or Outlier based on the rule:

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Form_1.PNG)

4. Count number of Inliers. If this is the largest number of inliers so far then update maximum and keep these inliers as bestInliers.
5. Repeat steps from 1. to 4. and stop after k iterations.

Since noise is randomly distributed, it is highly unlikely that the chosen model will fit to points that are not
inliers. With the gathered inlier points, one can estimate the transformation given the model.
Since the probability of success for finding a correct random sample of n points from where to instantiate
the model is given by 
<img src="https://render.githubusercontent.com/render/math?math=P=1-(1-p^n)^k">
, then, to estimate the number of iterations to use, we can use:

<img src="https://render.githubusercontent.com/render/math?math=k=\frac{\log(1-P)}{\log(1-p^n)}">

As an illustration, let us consider a model which needs a minimum number of points n=4. Table 1 contains the number of iterations, k, necessary to have a probability of P of getting at least one random sample of n points where all the sampled points are inliers (and where the probability of being inlier is p). As we can see, the number of iterations increases a lot when the probability of having inliers in the dataset decreases.

*Table 1: Number of RANSAC iterations as a function of p and P*

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Table_1.PNG)

### 1.5 Estimating transformation between two point clouds
The most general transformation between two rigid (as opposed to deforming) point clouds p and q is a three-dimensional rigid body transformation, composed of a rotation, R and a translation, T. The objective function we want to minimize is thus:

<img src="https://render.githubusercontent.com/render/math?math=E(R,T)=\sum_{i=1}^{N} \left \| \mathbf{q_i}-(R\mathbf{p_i}+\mathbf{T}) \right \|^2">

The first step in determining R and T that minimize Equation 4 is the determination of the centroids of the
point clouds and their subsequent subtraction from the original point clouds.

<img src="https://render.githubusercontent.com/render/math?math=q_{0}=q-\sum_{i=1}^{N} \frac{\mathbf{q_i}}{N}\quad,\quad p_{0}=p-\sum_{i=1}^{N} \frac{\mathbf{p_i}}{N}">

Then, it is possible to prove that the rotation matrix, Rp and translation vector, Tp that minimize the
optimization equation 4 is computed in the following way:

<img src="https://render.githubusercontent.com/render/math?math=M=p_{0}\,q_{0}^T=U\Sigma V^T \quad \quad (4)">

<img src="https://render.githubusercontent.com/render/math?math=\widehat{R}=VU^T \quad \quad (5)">

<img src="https://render.githubusercontent.com/render/math?math=\widehat{\mathbf{T}}=\sum_{i=1}^{N} \frac{\mathbf{p_i}}{N}-\widehat{R}\sum_{i=1}^{N} \frac{\mathbf{q_i}}{N}">

Where 
<img src="https://render.githubusercontent.com/render/math?math=U\Sigma V^T">
is the singular value decomposition of M. A detailed proof as to why these 
<img src="https://render.githubusercontent.com/render/math?math=\widehat{R}">
and 
<img src="https://render.githubusercontent.com/render/math?math=\widehat{\mathbf{T}}">
are the ones that minimize Equation 4 can be found in Appendix A.

### 1.6 The ICP algorithm
The Iterative Closest Point (ICP) algorithm’s purpose in this context is to minimize Equation 4 without knowing a priori what points are matching points in a pair of point clouds. To estimate which points are matches, it considers a sub-sample of the points in one point cloud, and determines the corresponding nearestneighbor in the other point cloud. Then, a fraction of the lowest-distanced neighbors are considered the matching points, with which it computes the transformation using the procedure described in Section 1.5. It then repeats this process until a stopping criteria has been achieved (i.e., minimum distance of nearest-neighbor, maximum
number of iterations). As long as the algorithm as a sufficiently close initial guess for the transformation, it converges.


## 2 Implementation
Having laid down the theoretical principals for this project on the previous Sections, we will now go through the sequence of steps taken to solve the proposed problem. A visual summary can be seen in Figure 3.

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Fig_3.PNG)

*Figure 3: Flowchart summarising the steps to solve the proposed problem*

The main body of the algorithm iterates over all consecutive images. Let’s call the 
<img src="https://render.githubusercontent.com/render/math?math=i^{th}">
RGB image
<img src="https://render.githubusercontent.com/render/math?math=RGB_i">
and its corresponding depth image
<img src="https://render.githubusercontent.com/render/math?math=D_i">.

The first sub-problem we need to address in this project is the combination of the data present in an RGB/depth image pair to include on the 3D scene. We can define the image coordinates *u* and *v* to simply be the indices of each pixel (with (0,0) coinciding with the top left corner), and Z to be the corresponding depth at those coordinates. We can then use Equation 3 with R “ I (identity matrix) and
<img src="https://render.githubusercontent.com/render/math?math=T=[0,0,0]^T">
(since we are still in the depth camera reference frame, and thus there is no geometric transformation of the coordinates apart from the pixel to meter conversion) to compute the X and Y coordinates. The corresponding equation is the following one, which is simply an inversion of Equation 3 with the specified *R* and 
<img src="https://render.githubusercontent.com/render/math?math=\mathbf{T}">.

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Eq_6.PNG)
<img src="https://render.githubusercontent.com/render/math?math=\quad \quad (6)">

Next, we need to transform the points from the depth camera reference frame to the RGB camera reference frame. These reference frames are related by a combination of a rotation and a translation. The corresponding equation is the following one:

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Eq_7.PNG)
<img src="https://render.githubusercontent.com/render/math?math=\quad \quad (7)">

Equations 6 and 7 are shown for a single point, but the computations have to be done for all points. Example
in Figure 4. In one iteration of the algorithm (as shown in Figure 3), this is performed for the consecutive pairs
<img src="https://render.githubusercontent.com/render/math?math=\{RGB_{i-1}, D_{i-1}\}">
<img src="https://render.githubusercontent.com/render/math?math=\{RGB_i, D_i\}">.

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Fig_4.PNG)

*Figure 4: Point cloud generation*

Then, using the SIFT algorithm, we extract the features and descriptors from the RGB images (properly converted into grayscale images). Then, we match the two sets of descriptors to find the indices in the images that correspond to common features. These matching points are, however, in RGB image coordinates, so they have to be transformed onto the 3D coordinates in the RGB camera reference frame.

The thresholds set for key point detection were decreased a lot to be able to find a lot of features. Since we have ways to exclude outliers, it is worth it to find as many features as possible, because correctly aligning 3D points is a very difficult procedure, and thus having as many points to work with as possible, will allow the detection of at least a few that are good to estimste the transformation.

Once we have two sets of matching 3D points, these are fed to the RANSAC algorithm for outlier detection. The model used for RANSAC is the general 3D affine model. The model is simply:

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Eq_8.PNG)
<img src="https://render.githubusercontent.com/render/math?math=\quad \quad (8)">

This model can be rewritten such that there is a closed-form solution. For each set of 3D points:

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Eq_9.PNG)
<img src="https://render.githubusercontent.com/render/math?math=\quad \quad (9)">

Which is an equation of the form:

<img src="https://render.githubusercontent.com/render/math?math=A\mathbf{h}=\mathbf{X} \quad \quad (10)">

Matrix A is repeated in rows with all other 3D points. With n=4 points, this is a determined system with solution:

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{h}=A^{-1}\mathbf{X} \quad \quad (11)">

If more than four points are used, this becomes an over-determined system, whose least squares solution is given by the Moore-Penrose pseudo-inverse:

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{h}=(A^{T}A)^{-1}A^T\mathbf{X} \quad \quad (12)">


For any case, matrix A is ill-conditioned when there are coplanar points. Ill-conditioned matrices don’t have well defined inverses, and thus the estimated parameters have a lot of error. To ensure this doesn’t influence the inlier choice, everytime the condition number for matrix A is larger than a threshold, that iteration stops
there and is repeated with a different permutation of four points.

For this project, this is the model fed to RANSAC. The reason why the more general affine model is chosen for outlier detection instead of the rigid model, is the fact that there is a closed-form solution for the parameters of the transformation, and since this is a more general model, the inliers will fit with this model as well. Since a lot of features are being detected by the SIFT algorithm, the number of iterations was set higher than usual, at k=2876. The distance threshold for inlier classification was set to 1 cm.

After the inliers have been found, these can be used to estimate the rotation and translation as described in section 1.5. One aspect we must turn our attention to is the fact that the rotation matrix can be estimated with det(R)=-1, which isn’t in fact a rotation, but a reflection. To account for this, the estimated rotation can then be corrected in the following way:

![](https://github.com/LeafarCoder/3D-point-cloud-registration/blob/master/Images/README/Eq_13.PNG)

Where the matrix between 
<img src="https://render.githubusercontent.com/render/math?math=V">
and
<img src="https://render.githubusercontent.com/render/math?math=U^T">
is the identity matrix except when
<img src="https://render.githubusercontent.com/render/math?math=VU^T">
is in fact a reflection, and the multiplication of this middle matrix corrects it to be a rotation (the final diagonal element will be a -1, the determinant of a reflection matrix).

With an initial estimate of *R* and 
<img src="https://render.githubusercontent.com/render/math?math=\mathbf{T}">
, we can now perform the ICP algorithm to refine this estimate. The fixed point cloud is the
<img src="https://render.githubusercontent.com/render/math?math=(i-1)^{th}">
th point cloud and the moving one will be the
<img src="https://render.githubusercontent.com/render/math?math=i^{th}">
one. The moving point cloud is also downsampled to achieve lower computation times and higher registration accuracy. Furthermore, during the ICP algorithm, sets of matching points are used to estimate the transformation if its Euclidean distances fall within a percentage set of matching distances. To assure that only the low matching distance points are used, this percentage is set to 20%. Otherwise, regions that are present in one point cloud but not in the other would be considered to estimate the transformation despite not being true matches, substantially influencing the transformation estimation in a negative way.

The ICP returns a final estimate of R and T, which can be used to merge the
<img src="https://render.githubusercontent.com/render/math?math=i^{th}">
point cloud with the current merge of all previous point clouds. Before doing so, the estimated transformation still has to be composed with the previous ones, since the estimated transformation was determined between clouds i and i-1, and we wish to merge them in the reference frame of the first point cloud. The composition can be exemplified for two sets of point clouds:

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{X_2}=R_{12}\mathbf{X_{1}}+\mathbf{T_{12}}">

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{X_3}=R_{23}\mathbf{X_{2}}+\mathbf{T_{23}}">

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{X_3}=R_{23}R_{12}\mathbf{X_{1}}+R_{23}\mathbf{T_{12}}+\mathbf{T_{23}}">

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{X_3}=R_{13}\mathbf{X_{1}}+\mathbf{T_{13}}\quad,\quad R_{13}=R_{23}R_{12}\quad,\quad \mathbf{T_{13}}=R_{23}\mathbf{T_{12}}+\mathbf{T_{23}} \quad \quad (14)">

This idea can be extended for any number of point clouds, ensuring that all clouds are transformed onto the reference frame of the first point cloud.

Every described step is then repeated for the next pair of consecutive point clouds, until all of them have been merged into a final one.
