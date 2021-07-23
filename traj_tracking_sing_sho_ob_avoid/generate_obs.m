function [x_obs,y_obs,x,y,r] = generate_obs(l_map,h_map)
%% function to generate an obstacle shaped as a circle with random center and radius
max_radius  =       0.8;
resolution  =       500;            %angular resolution
levels      =       80;             %radius resolution 

x           =       rand(1)*max(l_map,h_map);
y           =       rand(1)*max(l_map,h_map);
r           =       (rand(1)+0.15)*max_radius;
th          =       0:pi/resolution:2*pi;

for i=1:levels
    if(i~=1)
        w           =       abs(w-0.1);             %temp variable to decrease radius
        t           =       w * cos(th) + x;
        v           =       w * sin(th) + y;
        x_obs       =      [x_obs,t];
        y_obs       =      [y_obs,v];
    else 
        x_obs       =       r * cos(th) + x;
        y_obs       =       r * sin(th) + y;
        w           =       r;
    end
end

