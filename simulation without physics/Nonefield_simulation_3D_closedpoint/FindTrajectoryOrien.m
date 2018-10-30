clc
clear all
close all
addpath('stlTools')
R_roll=0;
R_pitch = 0;
R_yall=0;
R_Xoffset = 0;
R_Yoffset = 25;
R_Zoffset = 5;

P=[R_Xoffset,R_Yoffset,R_Zoffset,R_roll,R_pitch,R_yall];
PlotPosition=[10,100,1500,680]; %Position of the window [x,y,wigth width, height]

TrajSetpoint=[77.5,23,-77.5;77.5,99.2,-77.5;78.5,106.5,-77.5;82.5,111.5,-77.5;93.218,118.15,-77.5;112.54,120,-81.5;116.5,121.3,-90.5;115.51,121.2,-99.5;112.5,120.8,-103.5;77.5,111.5,-105;37.5,101.5,-105;28.48,96.5,-112.5;37.5,91.5,-120;77.5,81.5,-120;117.5,71.5,-120;126.5,66.5,-112.5;117.5,61.5,-105;77.5,51.5,-105;37.5,41.5,-105;28.48,36.5,-112.5;37.5,31.5,-120;77.5,21.5,-120;117.5,11.54,-120;126.83,11.31,-115.99;126.5,10.67,-102.47;121.5,9.97,-87.48;109.08,9.57,-79.05;100.5,9.5,-77.5;91.5,9.5,-77.5;88.15,9.7,-77.5;83.9,10.69,-77.5;80.9,12.57,-77.5;78.21,15.9,-77.5;77.5,18.5,-77.5;77.5,23,-77.5];
[m,n]=size(TrajSetpoint);
Traj = zeros(m,n);
Traj(:,1:n) = TrajSetpoint(:,:);
v = zeros(1000,3);
XLD=linspace(0,1,m);
XHD=linspace(0,1,1000);

Xtraj=interp1(XLD,Traj(:,1),XHD,'spline')-77.5;
Ytraj=interp1(XLD,Traj(:,2),XHD,'spline');
Ztraj=interp1(XLD,Traj(:,3),XHD,'spline')+77.5;
for i = 1:1000
    if i == 1000
        v(i,:) = [Xtraj(1)-Xtraj(i),Ytraj(1)-Ytraj(i),Ztraj(1)-Ztraj(i)]/norm([Xtraj(1)-Xtraj(i),Ytraj(1)-Ytraj(i),Ztraj(1)-Ztraj(i)]);
    else
        v(i,:) = [Xtraj(i+1)-Xtraj(i),Ytraj(i+1)-Ytraj(i),Ztraj(i+1)-Ztraj(i)]/norm([Xtraj(i+1)-Xtraj(i),Ytraj(i+1)-Ytraj(i),Ztraj(i+1)-Ztraj(i)]);
    end
end


figure(1)
for i = 1:1000
    %calculate robot roll pitch and yaw with deriction vector
    roll = 0;
    pitch =atan2(v(i,2),sqrt(v(i,3)^2+v(i,1)^2));
    yaw = atan2(v(i,1),v(i,3));
    x = cos(yaw)*cos(pitch);
    y = sin(yaw)*cos(pitch);
    z = sin(pitch);
    
    subplot(2,2,[1,3])
    cla
    plot3(Ztraj,Xtraj,Ytraj,'LineWidth',1)
    axis equal
    set(gca,'YDir','normal')
    set(gca,'ZDir','normal')
    set(gca,'XDir','normal')
    xlabel('X [mm]')
    ylabel('Y [mm]')
    zlabel('Z [mm]')
    set(gcf, 'Position', PlotPosition)
    hold on 
%     quiver3(Ztraj(i),Xtraj(i),Ytraj(i),v(i,3)',v(i,1)',v(i,2)','color','r')
    robot = stlread('robot.stl');
    p1 = patch(robot,'EdgeColor','none','FaceColor',[0,0,0],'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.6);
    R_X=[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)];
    R_Y = [cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)];
    R_Z=[cos(yaw-pi/2),-sin(yaw-pi/2),0;sin(yaw-pi/2),cos(yaw-pi/2),0;0,0,1];
    p1.Vertices=(R_Z*R_X*R_Y*p1.Vertices')'+ Ztraj(i).*repmat([1,0,0],[size(robot,1),1])+Xtraj(i).*repmat([0,1,0],[size(robot,1),1])+Ytraj(i).*repmat([0,0,1],[size(robot,1),1]);
    camlight
    view([90,90,90])
    
    drawnow

    subplot(2,2,2)
    cla
    grid on    

    hold on
    quiver3(0,0,0,5*x,5*y,5*z,'color','g')
    quiver3(0,0,0,10*v(i,3)',10*v(i,1)',10*v(i,2)','color','r')
    
    robot = stlread('robot.stl');
    p1 = patch(robot,'EdgeColor','none','FaceColor',[0,0,0],'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.6);
    R_X=[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)];
    R_Y = [cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)];
    R_Z=[cos(yaw-pi/2),-sin(yaw-pi/2),0;sin(yaw-pi/2),cos(yaw-pi/2),0;0,0,1];
    p1.Vertices=(R_Z*R_X*R_Y*p1.Vertices')'+ 0.*repmat([1,0,0],[size(robot,1),1])+0.*repmat([0,1,0],[size(robot,1),1])+0.*repmat([0,0,1],[size(robot,1),1]);
    axis([-5,5,-5,5,-5,5])
    camlight
    view([90,90,90])
    drawnow
    
    subplot(2,2,4)
    cla
    grid on    

    hold on
    quiver3(0,0,0,5*x,5*y,5*z,'color','g')
    quiver3(0,0,0,10*v(i,3)',10*v(i,1)',10*v(i,2)','color','r')
    
    robot = stlread('robot.stl');
    p1 = patch(robot,'EdgeColor','none','FaceColor',[0,0,0],'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.6);
    R_X=[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)];
    R_Y = [cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)];
    R_Z=[cos(yaw-pi/2),-sin(yaw-pi/2),0;sin(yaw-pi/2),cos(yaw-pi/2),0;0,0,1];
    p1.Vertices=(R_Z*R_X*R_Y*p1.Vertices')'+ 0.*repmat([1,0,0],[size(robot,1),1])+0.*repmat([0,1,0],[size(robot,1),1])+0.*repmat([0,0,1],[size(robot,1),1]);
    axis([-5,5,-5,5,-5,5])
    camlight
    view([90,90,90])
    drawnow
    pause(0.05)
end

