% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
M = size(scanAngles,1);
for j = 1:N % for each time
   for theta = 1:M
      x_o = ranges(theta,j) * cos(scanAngles(theta,1) + pose(3,j)) + pose(1,j);
      y_o = -1*ranges(theta,j) * sin(scanAngles(theta,1) + pose(3,j)) + pose(2,j);
     
      % Find occupied-measurement cells and free-measurement cells
      occ= [ceil(x_o*myResol)+ myorigin(1)  ceil(y_o*myResol) + myorigin(2)];
      
      current = [ceil(pose(1,j)*myResol) + myorigin(1)  ceil(pose(2,j)*myResol) + myorigin(2)];
      
      if occ(1)>0 && occ(2)>0 &&  occ(2) < param.size(1)+1 &&  occ(1) < param.size(2)+1
        myMap(occ(2),occ(1)) =  myMap(occ(2),occ(1))  + lo_occ;
      end
      [freex,freey]  = bresenham(current(1),current(2),occ(1),occ(2));  
      if size(freex,2)>0
        freex_ = freex';
        freey_ = freey';
        del_index = freex_<1 | freex_> param.size(2) | freey_<1 | freey_>param.size(1);
        freex_(del_index) = [];
        freey_(del_index) = [];
        freex = freex_';
        freey = freey_';
        free = sub2ind(size(myMap),freey,freex);
        myMap(free) = myMap(free)-lo_free;
      end
   end
   myMap = min(myMap,lo_max);
   myMap = max(myMap,lo_min);
end

end

