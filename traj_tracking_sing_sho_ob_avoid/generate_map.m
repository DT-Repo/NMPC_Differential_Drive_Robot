function [x_obs,y_obs,x_c,y_c,r] = generate_map(w_map,h_map,n_obs)
%% function to generate a map with a certain number of obstacles
% obstacles properties
resolution  =       500;                            %angular resolution
samples     =       length(0:pi/resolution:2*pi);   %density of points for a given radius
levels      =       80;                             %radius resolution

x_obs       =       zeros(samples*levels,n_obs);    %x-points belong to obstacles except center
y_obs       =       zeros(samples*levels,n_obs);    %y-points belong to obstacles except center
x_c         =       zeros(n_obs,1);
y_c         =       zeros(n_obs,1);                 %center coordinates and radius
r           =       zeros(n_obs,1);
% generate obstacles
for i = 1:n_obs
    [x_obs(:,i),y_obs(:,i),x_c(i,1),y_c(i,1),r(i,1)]   =   generate_obs(w_map,h_map);   
end

end

