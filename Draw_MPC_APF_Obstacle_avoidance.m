function Draw_MPC_APF_Obstacle_avoidance (t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_cl,obs_diam,mass, rx,ry,tol,factor)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 2.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

n_rects = size(rx,1);
lanes = zeros(n_rects,n_rects);
lanes(:,1) = rx(:,1) ; lanes(:,3) = abs(rx(:,2) - rx(:,1)) ;
lanes(:,2) = ry(:,1) ; lanes(:,4) = abs(ry(:,2) -ry(:,1));

disp(lanes)

n_obstacles = length(obs_x);

r = rob_diam/2;  % Robot radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);


figure(500)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

tic
for k = 1:size(xx,2)
    h_t = 0.14; w_t=0.09; % triangle parameters
    
    x1 = xs(1); y1 = xs(2); th1 = xs(3);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    th = 0:0.01:2*pi;
    x_cir = x1+ tol*cos(th);
    y_cir = y1+ tol*sin(th);
    plot(x_cir,y_cir,'k--','LineWidth',2);
    % plot(x_ref(1,:),x_ref(2,:),'b--','LineWidth',2);
    hold on;
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width); hold on % plot exhibited trajectory
    if k < size(xx,2) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
        % plot(obs_cl(1:N,1,k),obs_cl(1:N,2,k),'b--*')
        for j = 2:N+1
            r = (obs_diam/2)*(factor)^j;  % obstacle radius
            xp_obs=r*cos(ang);
            yp_obs=r*sin(ang);
            plot(xx1(j,1,k)+xp,xx1(j,2,k)+yp,'--r','linewidth',line_width*0.25); % plot robot circle
            plot(obs_cl(j,1,k)+xp_obs,obs_cl(j,2,k)+yp_obs,'--b','linewidth',line_width*0.5)
        end
    end
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    plot(x1+xp,y1+yp,'r', 'linewidth',line_width); % plot robot circle
    
    % for n = 1: n_obstacles
    %     r = (obs_diam/2)*(factor)^j;
    %     xp_obs=r*cos(ang);
    %     yp_obs=r*sin(ang);
    %     plot(obs_x(n)+xp_obs,obs_y(n)+yp_obs,'--b'); % plot obstacle circle 1   
    % end
    
    for n_lanes = 1:size(lanes,1)
        rectangle('Position',lanes(n_lanes,:),'FaceColor',[0 0 0],'Edgecolor','b',...
            'LineWidth',3)
    end
%     P = [p1;p2;p3;p4];
%     plot(P(:,1),P(:,2),'k','linewidth',2)
% %     plot(P(:,3),P(:,4),'k','linewidth',2)

    hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-5.0  7.0 -5 7])
    pause(0.1)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
end

toc
close(gcf)
%viobj = close(aviobj)
% video = VideoWriter('C:\Users\n2kan\OneDrive - Virginia Tech\Anshul\Research\Ph.D\Uncertainty_aware_planning\NMPC\Videos\Tracking_with_1_obs.avi','Uncompressed AVI');
% video.FrameRate = 10;  % (frames per second) this number depends on the sampling time and the number of frames you have
% open(video)
% writeVideo(video,F)
% close (video)

figure
subplot(211)
stairs(t,u_cl(:,1)/mass,'k','linewidth',1.5); axis([0 t(end) -1 1])
ylabel('a (m/s^2)')
grid on
subplot(212)
stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -1.57*0.2 1.57*0.2])
xlabel('time (seconds)')
ylabel('\omega (rad/s)')
grid on
f = gcf;
print(f,'.\Trajectory_tracking.png','-dpng','-r300');


