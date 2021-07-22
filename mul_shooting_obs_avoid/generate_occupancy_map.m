function map = generate_occupancy_map(w_map,h_map,n_obs,x,y,xc,yc,rad)
%% function to discretize the continuos map and scale it to trasform it into an occupancy map
scale_factor = 1;
scaled_w_map = scale_factor * w_map;
scaled_h_map = scale_factor * h_map;

x = round(x,1);
y = round(y,1);
logic_map = logical(zeros(scaled_h_map,scaled_w_map));
[n,~]=size(x);
for i=1:n
    for j=1:n_obs
        if(x(i,j)<w_map && y(i,j)<h_map && x(i,j)>0 && y(i,j)>0 )
            info_y=ceil(y(i,j)*scale_factor);
            info_x=ceil(x(i,j)*scale_factor);
            if(info_x<scaled_w_map && info_y<scaled_h_map)
                logic_map(scaled_h_map-info_y,info_x)=true;
            end
        end
    end
end
        
map = occupancyMap(logic_map,scale_factor);
end
