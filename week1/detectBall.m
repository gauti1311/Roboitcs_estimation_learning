% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = [151.9007  145.6434   58.1984]';
cov = [148.4629   98.7787 -154.7730;
   98.7787  122.4508 -151.2728;
 -154.7730 -151.2728  290.3352];

th = 1/((2*pi)^1.5*det(cov)^0.5);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
% 
I_ = double(I);
p = zeros(size(I_,1),size(I_,2));
for n=1:size(I_,1)
    for m=1:size(I_,2)
        I1 = I_(n ,m,:);
        d = I1(:)-mu;
        p(n,m) = exp(-0.5*d'*((cov)\d))/((2*pi)^1.5*det(cov)^0.5);
    end
end        
segI = p>th/10; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
CC = bwconncomp(segI);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;

% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
