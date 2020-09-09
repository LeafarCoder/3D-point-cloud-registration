%% PIV Project
load('sinteticotable.mat');
%% Organize data
% The folders function must receive as an input a folder in the current 
% working directory that has folders inside it. These 2nd level folders
% should have the images:

% master_folder
% |
% |___ folder1
% |     |__ 1_image_1
% |     |__ ...
% |     |__ 1_image_n
% |
% |___ folder2
% |     |__ 2_image_1
% |     |__ ...
% |     |__ 2_image_m
% |
% |___ folder3
%       |__ 3_image_1
%       |__ ...
%       |__ 3_image_k

% Note: The names of the folders and images inside them do not need to be 
% these ones, they can be any name you'd like.

% This creates a (n_datasets x 1) cell, whose elements are also cells.
% These 2nd level cells are (n_images x 1) cells, that hold the name of the
% directory of the image:


list=find_image_lists('datasets');

%% Return mosaic
% This function builds the mosaic and displays it, as well as returning a 
% (n_images x n_images) cell that contains the homographies between image i
% and j in each (i,j) position of the cell - The diagonal is composed of
% identity matrices and position (j,i) has the inverse of (i,j).

[H]=part1(list{3},match_list);
 
% return corners-
% hold on
% plot(cx(:),cy(:), 'r+', 'MarkerSize', 10, 'LineWidth', 2);

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%  DEFINING FUNCTIONS  %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% find_image_lists

function [list] = find_image_lists(str)
list={};
files=dir(str);
j=1;
for file =files'
    if ~contains(file.name, '.')
        im_list={};
        im_files=dir(strcat(str,'/',file.name));
        i=1;
        for im_file =im_files'
            if contains(im_file.name, 'jpg') || contains(im_file.name, 'png')
                im_list(i)={strcat(str,'/',file.name,'/',im_file.name)};
                i=i+1;
            end
        end


        list(j)={im_list};

        j=j+1;
    end
end

end

%% part1

% Check input format
function [varargout] = part1(varargin)
if nargin<2
	error('Please input both image_list and match_list');
    exit;
elseif nargin==2
    image_list=varargin{1};
    match_list=varargin{2};
elseif nargin==3
    error('Please provide both K and pointsw as inputs');
    exit;
elseif nargin==4
    image_list=varargin{1};
    match_list=varargin{2};
    K=varargin{3}; 
    pointsw=varargin{4}; 
elseif nargin>4
    error('Too many input arguments');
    exit;
end
    
% Number of images
n_images=size(image_list,2);

% Initialize the chain of transformations for consecutive images
chain_transfs = cell(n_images,1);
chain_transfs{1} = eye(3);

% Initialize the transformations matrix: transformation from image i to j
H = cell(n_images,n_images);
H{1,1} = eye(3);

% In case images have different sizes (width and height)
imageSizes = zeros(n_images,2);
I = imread(image_list{1});
sz = size(I);
imageSizes(1,:) = sz(1:2);

% Use more points than the ones given by using 'vlfeat-0.9.21' algorithm
matches = cell(n_images-1,2);

% Getting feature points for image 1
f = waitbar(0,'Getting matching points for image 1...');
grayImage = single(rgb2gray(I));
[f2,d2]=vl_sift(grayImage);

% Iterate over remaining image pairs
for i = 2:n_images

    f1=f2;
    d1=d2;
    
    I = imread(image_list{i});
    sz=size(I);
    imageSizes(i,:)=sz(1:2);
    
    waitbar((i)/n_images,f,['Getting matching points for image ',num2str(i+1),'...']);
    grayImage = single(rgb2gray(I));    
    [f2,d2]=vl_sift(grayImage);
    if(i == n_images-1)
        close(f);   
    end
    
    % Combine matching points from two consecutive images
    match = vl_ubcmatch(d2,d1);
    X1 = f1(1:2,match(2,:)) ;
    X2 = f2(1:2,match(1,:)) ;
    matches{i-1,1} = X1';
    matches{i-1,2} = X2';
    
    transf = ransac(matches{i-1,2},matches{i-1,1});
    transf_obj = projective2d(transf);
    
    % Calculate transformation from i to all previous images
    H{i,i} = eye(3);
    for j = i-1:-1:1
        H{j,i} = H{j,i-1} * transf;
        H{i,j} = inv(H{j,i});
    end

    chain_transfs{i} = transf' * chain_transfs{i-1};
end

H=H';

xlim=zeros(n_images,2);
ylim=zeros(n_images,2);

% Get Limits for the image
for i = 1:numel(chain_transfs)
    [xlim(i,:), ylim(i,:)] = outputLimits(projective2d(chain_transfs{i}), [1 imageSizes(i,2)], [1 imageSizes(i,1)]);    
end

% mean limit for all transforms
avgXLim = mean(xlim, 2);

% Recover indexes from the sorted mean limits
[~, idx] = sort(avgXLim);

centerIdx = floor((numel(chain_transfs)+1)/2);

% get the index of the image that has the median mean of xlimits. This
% issentially picks the image in the "center" of the mosaic, and uses it to
% be the planar portion of the mosaic, with the images to the left curving
% leftward and the images to the right curving rightward from this center
% image
centerImageIdx = idx(centerIdx);

% Make the center image the planar one by "removing" its transformation 
% from the remaining ones, and ensuring that this center one has the 
% identity in tforms
for i = 1:numel(chain_transfs)
    chain_transfs{i} = chain_transfs{i} / chain_transfs{centerImageIdx};
end

% Create output limits for the canvas in which the images will be placed
for i = 1:numel(chain_transfs)           
    [xlim(i,:), ylim(i,:)] = outputLimits(projective2d(chain_transfs{i}), [1 imageSizes(i,2)], [1 imageSizes(i,1)]);
end

maxImageSize = max(imageSizes);

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of canvas.
width  = ceil(xMax - xMin);
height = ceil(yMax - yMin);

% Initialize the "empty" canvas.
canvas=uint8(zeros([height width 3]));

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

figure(1);
% Create the panorama.
corner_pts_x=zeros(4,n_images);
corner_pts_y=zeros(4,n_images);

for i = 1:n_images
    I = imread(image_list{i});   
    
    % Transform I into the panorama.
    warpedImage = imwarp(I, projective2d(chain_transfs {i}), 'OutputView', panoramaView);
               
    % Generate a binary mask.   
    mask = imwarp(true(size(I,1),size(I,2)), projective2d(chain_transfs{i}), 'OutputView', panoramaView);
    if nargin==4
        c=corners(mask.*256);
        corner_pts_x(:,i)=c(:,2);
        corner_pts_y(:,i)=c(:,1);
    end
    % Overlay the warpedImage onto the panorama.
    canvas = step(blender, canvas, warpedImage, mask);
    imshow(canvas)
    %hold on
    %plot(corner_pts_x(:,i),corner_pts_y(:,i), 'r+', 'MarkerSize', 10, 'LineWidth', 2);
    pause(0.5)
end

 world_pts_x=zeros(n_images,4);
 world_pts_y=zeros(n_images,4);

w=homography(match_list{1,1},pointsw); 
W=[w(1) w(2) w(3); w(4) w(5) w(6); w(7) w(8) 1];
for i=1:n_images
    hij=W*H{i,1};
    ul=[1 1];
    ur=[imageSizes(i,2) 1];
    dr=[imageSizes(i,2) imageSizes(i,1)];
    dl=[1 imageSizes(i,1)];
    
    ult1=(ul(1)*hij(1,1)+ul(2)*hij(1,2)+hij(1,3))/(ul(1)*hij(3,1)+ul(2)*hij(3,2)+hij(3,3));
    ult2=(ul(1)*hij(2,1)+ul(2)*hij(2,2)+hij(2,3))/(ul(1)*hij(3,1)+ul(2)*hij(3,2)+hij(3,3));
    
    urt1=(ur(1)*hij(1,1)+ur(2)*hij(1,2)+hij(1,3))/(ur(1)*hij(3,1)+ur(2)*hij(3,2)+hij(3,3));
    urt2=(ur(1)*hij(2,1)+ur(2)*hij(2,2)+hij(2,3))/(ur(1)*hij(3,1)+ur(2)*hij(3,2)+hij(3,3));
    
    dlt1=(dl(1)*hij(1,1)+dl(2)*hij(1,2)+hij(1,3))/(dl(1)*hij(3,1)+dl(2)*hij(3,2)+hij(3,3));
    dlt2=(dl(1)*hij(2,1)+dl(2)*hij(2,2)+hij(2,3))/(dl(1)*hij(3,1)+dl(2)*hij(3,2)+hij(3,3));
     
    drt1=(dr(1)*hij(1,1)+dr(2)*hij(1,2)+hij(1,3))/(dr(1)*hij(3,1)+dr(2)*hij(3,2)+hij(3,3));
    drt2=(dr(1)*hij(2,1)+dr(2)*hij(2,2)+hij(2,3))/(dr(1)*hij(3,1)+dr(2)*hij(3,2)+hij(3,3));
    
    world_pts_x(i,1:4)=[ult1 urt1 drt1 dlt1];
    world_pts_y(i,1:4)=[ult2 urt2 drt2 dlt2];   
end

if nargin==2
    varargout{1}=H;
else
    varargout{1}=H;
    varargout{2}=world_pts_x;
	varargout{3}=world_pts_y;
    varargout{4}=corner_pts_x';
    varargout{5}=corner_pts_y';
end

end

%% homography

function [h] = homography(pts1,pts2)
if(size(pts1,1)~=size(pts2,1))
    error("The number of points isn't the same for both vectors");
    exit;
elseif size(pts1,1)<4
    error('Ill-posed problem. Please provide 4 points or more.');
    exit;
end

n_pts=size(pts1,1);

A=zeros(n_pts*2,8);

b=pts2';
b=b(:);

for i=1:n_pts
    A(2*i-1,:)=[pts1(i,1), pts1(i,2), 1, 0, 0, 0, -pts1(i,1)*pts2(i,1), -pts1(i,2)*pts2(i,1)];
    A(2*i,:)=[0, 0, 0, pts1(i,1), pts1(i,2), 1, -pts1(i,1)*pts2(i,2), -pts1(i,2)*pts2(i,2)];
end

h=pinv(A)*b;

end

%% ransac

function [H] = ransac(p1,p2)

% Define initialization variables
p=0.99;
N=ceil(log(1-p)/log(1-(0.15)^4));
min_dist=.4;
max_inlier=-1;
min_std=Inf;

p1=double(p1);
p2=double(p2);
    
for j=1:N
    idx=randperm(numel(p1(:,1)),4);
    h=homography(p1(idx,:),p2(idx,:));
    p2_h=zeros(size(p2));
    for i=1:size(p2_h,1)
        p2_h(i,1)=(p1(i,1)*h(1)+p1(i,2)*h(2)+h(3))/(p1(i,1)*h(7)+p1(i,2)*h(8)+1);
        p2_h(i,2)=(p1(i,1)*h(4)+p1(i,2)*h(5)+h(6))/(p1(i,1)*h(7)+p1(i,2)*h(8)+1);
    end

    sub=p2_h-p2;
    d=sqrt(sub(:,1).^2+sub(:,2).^2);

    inliers=[];
    for l=1:length(d)
        if d(l)<min_dist
            inliers=[inliers l];
        end
    end

    m=length(inliers);
    curr_std=std(d);
    if m>max_inlier || (m==max_inlier && curr_std<min_std)
        good_matches=inliers;
        max_inlier=m;
        min_std=curr_std;
    end

end
p1=p1(good_matches,:);
p2=p2(good_matches,:);
hf=homography(p1,p2);

H=[hf(1) hf(2) hf(3); hf(4) hf(5) hf(6); hf(7) hf(8) 1];
end

%% corners

function [c] = corners(v)
N=360;
theta=linspace(0,360,N);
[I,J]=find(v);
IJ=[I,J];
c=nan(size(theta));
for i=1:N 
    [~,c(i)]=max(IJ*[cosd(theta(i));sind(theta(i))]);  
end
H=histcounts(c,1:numel(I)+1);
[~,k] = maxk(H,4);
c=IJ(k,:);
end
