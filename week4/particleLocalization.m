% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
%n = size(scanAngles, 1);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% % the number of grids for 1 meter.
R = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;

% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 1000;                      % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
Weights = ones(1,M) * (1/M); 

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    
     if j < 30
         noise_cov = diag([0.1 0.1 0.035]);
     else
         M = 500 ;
         noise_cov = diag([0.025 0.025 0.03]);
     end
    noise_mu = [0 0 0];
    
    for m = 1:M
      % 1) Propagate the particles
        P(:,m) = myPose(:,j-1) +  mvnrnd(noise_mu,noise_cov)';
        w = 0;
%      2) Measurement Update 

%      2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
         x_o = ranges(:,j) .* cos(scanAngles + P(3,m)) + P(1,m);
         y_o = -ranges(:,j) .* sin(scanAngles + P(3,m)) + P(2,m);
            
         occ_x = (ceil(x_o*R) + myOrigin(1))';
         occ_y = (ceil(y_o*R) + myOrigin(2))';
         
         del_occ =  occ_x<1 | occ_y<1 |  occ_x > size(map,2) |  occ_y > size(map,1);

         occ_x(del_occ) = [];
         occ_y(del_occ) = [];

         occ_index = sub2ind(size(map),occ_y',occ_x');
          
%        2-2) For each particle, calculate the correlation scores of the particles
        
         w =  w + sum(sum(map(occ_index) >= 0.7)) * 10;
         w =  w - sum(sum(map(occ_index) < -0.2)) * 2;
  
%       2-3) Update the particle weights         
      
         Weights(1,m) = w;  
    end
    
%       2-4) Choose the best particle to update the pose

        Weights = Weights/sum(Weights);
        [~,Id] = max(Weights);
        myPose(:,j) = P(:,Id);
        
%     % 3) Resample if the effective number of particles is smaller than a threshold
        n_eff = sum(Weights) * sum(Weights) / sum(Weights)^2;

%     % 4) Visualize the pose on the map as needed

end