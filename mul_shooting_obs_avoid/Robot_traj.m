function Robot_traj (t,xx,xx1,u_cl,xs,N,rob_diam,x_,y_,w_map,h_map,th)
% Parameter
r = th(1,1);
d = th(2,1);

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];



r = rob_diam/2;  % obstacle radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

figure(5)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(xx,2)
    h_t = 0.24; w_t=0.19; % triangle parameters
    
    x1 = xs(1); y1 = xs(2); th1 = xs(3);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(xx,2) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    end
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    plot(x1+xp,y1+yp,'--r'); % plot robot circle
    view_map(x_,y_,w_map,h_map);
   
    hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([0 10 0 8])
    pause(0.1)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
end
% close(gcf)
% viobj = close(aviobj)
% video = VideoWriter('exp.avi','Uncompressed AVI');
% 
% video = VideoWriter('exp.avi','Motion JPEG AVI');
% video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
% open(video)
% writeVideo(video,F)
% close (video)

figure
subplot(221)
stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -0.8 0.8])
ylabel('v (m/s)')
xlabel('time (seconds)')
grid on
subplot(222)
stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -2 2])
xlabel('time (seconds)')
ylabel('\omega (rad/s)')
grid on
subplot(223)
omega_R = (u_cl(:,1) + u_cl(:,2)*(d/2))/r;
stairs(t,omega_R,'k','linewidth',1.5); axis([0 t(end) -8 8])
ylabel('\omega_R (rad/s)')
xlabel('time (seconds)')
grid on
subplot(224)
omega_L = (u_cl(:,1) - u_cl(:,2)*(d/2))/r;
stairs(t,omega_L,'k','linewidth',1.5); axis([0 t(end) -8 8])
ylabel('\omega_L (rad/s)')
xlabel('time (seconds)')
grid on
