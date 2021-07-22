function []=view_map(x_obs,y_obs,w_map,h_map)
%% function to visualize a continuos map
[~,n_obs]=size(x_obs);

plot(x_obs,y_obs);
axis([0 w_map 0 h_map]);



