%/////////////////////////////////////////////////////////////////////////
%This program was made by Haoran and Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 9/27/2018

%This script is an algorithm simulating the functioning of a 3D magnetic
%manipulator. The system manipulates a cylinder magnet in the X-Y-Z plane. The cylinder
%revolution axis is in the same plane. The system has 6 electromagnets
%arranges in a square shape.

%Clear out everything
close all
clear
image_index =1;
start =1;
%add folder path
addpath('stlTools')
addpath('complete Controller')
addpath('Filters')
addpath('Measurement Simulation')
addpath('Trajectory controllers')

%////////////////////////////////////////////////////////////////////////
%/////////////Begining of variables and constant definition//////////////
%Variables used to control the figures
%These values can be modified by the user
PlotPosition=[10,100,1500,680]; %Position of the window [x,y,wigth width, height]
Plot_Side_Length=0.15;% [m]%Length to plot on each axis of the actuation matrix conditioning map
PlotConditioning=false; % Only plot conditioning when true. Set to false to dislabe this plot and speed up the computation
NbPtsTimePlot=20; %Number of time points to plot for the plots as function of time.

R_roll=0;
R_pitch = 0;
R_yaw=0;
R_Xoffset = 0.02;
R_Yoffset = 0.025;
R_Zoffset = -0.058;
Orien = [cos(R_yaw)*cos(R_pitch), sin(R_yaw)*cos(R_pitch), sin(R_pitch)];
P=[R_Xoffset,R_Yoffset,R_Zoffset]; %Initial position of the robot [ Xposition [m] , Yposition [m], roll[rad], pitch[rad], yall[rad]]
V=[0,0,0]; %Velocity of the robot[ Xvelocity [m/s] , Yvelocity [m/s], Zvelocity, Roll Velocity [rad/s], Pitch Velocity [rad/s], Yaw Velocity [rad/s]]

%Parameters of the controller
%These values can be modified by the user
HistoryErrorVelocityLegnth=300; %Number of points in the vector HistoryErrorVelocity. This vector is used to compute the integral component of the Velocity controller. It has a finite length and therfore allow forgeting old errors.

DistanceHistory = zeros(1003,1);
PositionHistory = zeros(1003,3);
%Other Constants definition
%These values can be modified by the user
T=0.31;%[m] length of the side of the cube formed by the coils.
Robot_Length=2e-3;%Length of the cylinder magnet[m]
Robot_diameter=0.75e-3;%[m] Diameter of the cylinder magnet[m]
RobotDensity=8000; %[Kg/m^3] Density of the robot, 8000 for Neodium magnets
CdForce=0.0015; %[N.s/m] Drag coefficient used in the controller internal model
CdTorque=0.00000005;
ControllerSpeedFactor=3; %[]
deltaT=0.01; %[s] Time step for physics simulation

%Program Variables initialization, modify at your own risk.
time=0; %starting time.
LastTime=time-deltaT;
DragCoef=[CdForce,CdForce,CdForce];  % why in this form?
TimeVector=0; %Time vector initialization. Used for plot
VelocitySetpointVector=[0;0]; %Velocity Setpoint vector initialization. Used for plot
VelocityVector=[0;0;0]; %Velocity vector initialization. Used for plot
HistoryErrorVelocity=zeros(3,HistoryErrorVelocityLegnth); %Initialize HistoryErrorVelocity
PhysicsIterNumber=0; %iteration number of physics simulation
NewErrorCovMatrix=eye(6,6); %Initialization of the covariance matrix used by the Kalman Filter
I=[0;0;0;0;0;0]; %Initialization of the current [0;0;0;0;0;0;]
StateVector=[0;0;0]; %Initialization of the state vector
MeasuredPositionVector=[0;0;0]; %Initialization of the measured position vector
RealPositionVector=[0;0;0]; %Initialization of the real position vector
NewState=[P(1);P(2);P(3);0;0;0]; %Initialize robot initial state
NewPositionMeasured=true; %Tell the controller a new position was measured

%%%%calculate high resolution trajectory and initial orientation for each
%%%%way point(roll pitch yaw)
TrajSetpoint=[77.5,23,-77.5;77.5,99.2,-77.5;78.5,106.5,-77.5;82.5,111.5,-77.5;93.218,118.15,-77.5;112.54,120,-81.5;116.5,121.3,-90.5;115.51,121.2,-99.5;112.5,120.8,-103.5;77.5,111.5,-105;37.5,101.5,-105;28.48,96.5,-112.5;37.5,91.5,-120;77.5,81.5,-120;117.5,71.5,-120;126.5,66.5,-112.5;117.5,61.5,-105;77.5,51.5,-105;37.5,41.5,-105;28.48,36.5,-112.5;37.5,31.5,-120;77.5,21.5,-120;117.5,11.54,-120;126.83,11.31,-115.99;126.5,10.67,-102.47;121.5,9.97,-87.48;109.08,9.57,-79.05;100.5,9.5,-77.5;91.5,9.5,-77.5;88.15,9.7,-77.5;83.9,10.69,-77.5;80.9,12.57,-77.5;78.21,15.9,-77.5;77.5,18.5,-77.5;77.5,23,-77.5];
[m,n]=size(TrajSetpoint);
Traj = zeros(m,n+1);
Traj(:,1:n) = TrajSetpoint(:,:);
Traj(:,n+1) = ones(m,1)*0.01;
orientation = zeros(1000,3);
XLD=linspace(0,1,m);
XHD=linspace(0,1,1000);

Xtraj=interp1(XLD,Traj(:,1),XHD,'spline');
Xtraj = Xtraj - (max(Xtraj)+min(Xtraj))/2;

Ytraj=interp1(XLD,Traj(:,2),XHD,'spline');
Ytraj = Ytraj - (max(Ytraj)+min(Ytraj))/2;

Ztraj=interp1(XLD,Traj(:,3),XHD,'spline');
Ztraj = Ztraj - (max(Ztraj)+min(Ztraj))/2;
for i = 1:1000
    if i == 1000
        orientation(i,:) = [Xtraj(1)-Xtraj(i),Ytraj(1)-Ytraj(i),Ztraj(1)-Ztraj(i)]/norm([Xtraj(1)-Xtraj(i),Ytraj(1)-Ytraj(i),Ztraj(1)-Ztraj(i)]);
    else
        orientation(i,:) = [Xtraj(i+1)-Xtraj(i),Ytraj(i+1)-Ytraj(i),Ztraj(i+1)-Ztraj(i)]/norm([Xtraj(i+1)-Xtraj(i),Ytraj(i+1)-Ytraj(i),Ztraj(i+1)-Ztraj(i)]);
    end
end

%Calculate constants used in the program
RobotMass=pi.*((Robot_diameter./2).^2).*Robot_Length.*RobotDensity; %[Kg] Mass of the robot


distance = MinimumDistance (P, Traj(:,1:3));
DistanceHistory(PhysicsIterNumber+1) = distance;

%/////////////End of variables and constant definition//////////////
%////////////////////////////////////////////////////////////////////////

%/////////////////////////////////////////////////////////////////////////
%Compute 3D map of the magnetic field produced by each coil for one amp.
%Used for plot during compuation

xmin=-T/2.*1; %[m] minimum x plot value
xmax=T/2.*1; %[m] maximum x plot value
ymin=-T/2.*1; %[m] minimum y plot value
ymax=T/2.*1; %[m] maximum y plot value
zmin = 0; % [m] minimum z plot value
zmax = T.*1; %[m] maximum z plot value
NbPtsX=50; %Numbers of points to plot on x axis for flux density map Must be a multiple of 10 top plot quiver correctly
NbPtsY=50; %Numbers of points to plot on y axis for flux density map Must be a multiple of 10 top plot quiver correctly
NbPtsZ=50; %Numbers of points to plot on y axis for flux density map Must be a multiple of 10 top plot quiver correctly
NormB=zeros(NbPtsX,NbPtsY,NbPtsZ);


Xplot=linspace(xmin,xmax,NbPtsX); %Initialize x vector
Yplot=linspace(ymin,ymax,NbPtsY); %Initialize y vector
Zplot=linspace(zmin,zmax,NbPtsZ); %Initialize y vector
[Xg,Yg,Zg] = meshgrid(Xplot,Yplot,Zplot);

% XplotQuiver=linspace(xmin,xmax,NbPtsX./50); %Initialize x vector
% YplotQuiver=linspace(ymin,ymax,NbPtsY./50); %Initialize y vector
% ZplotQuiver=linspace(zmin,zmax,NbPtsZ./50); %Initialize y vector
% [XQ,YQ,ZQ] = meshgrid(XplotQuiver,YplotQuiver,ZplotQuiver);
%
% for i=1:NbPtsX
%     if ~mod(i,10)
%         clc
%         disp('Computing flux density map. ')
%         disp(['Progress:',num2str(100.*i/NbPtsX),' %'])
%     end
%     for j=1:NbPtsY
%         for k = 1:NbPtsZ
%             MatPlotB(i,j,k,:,:)=[GFunction( 1,[Xplot(i),Yplot(j),Zplot(k)],coil_radius,T );GFunction( 2,[Xplot(i),Yplot(j),Zplot(k)],coil_radius,T );GFunction( 3,[Xplot(i),Yplot(j),Zplot(k)],coil_radius,T );GFunction( 4,[Xplot(i),Yplot(j),Zplot(k)],coil_radius ,T); GFunction( 5,[Xplot(i),Yplot(j),Zplot(k)],coil_radius,T );GFunction( 6,[Xplot(i),Yplot(j),Zplot(k)],coil_radius,T )]  ;       %#ok<*SAGROW>
%         end
%     end
% end
% for i=1:NbPtsX./50
%     for j=1:NbPtsY./50
%         for k = 1: NbPtsZ./50
%         MatPlotBQuiver(i,j,k,:,:)=[GFunction( 1,[XplotQuiver(i),YplotQuiver(j),ZplotQuiver(k)],coil_radius,T );GFunction( 2,[XplotQuiver(i),YplotQuiver(j),ZplotQuiver(k)],coil_radius,T );GFunction( 3,[XplotQuiver(i),YplotQuiver(j),ZplotQuiver(k)],coil_radius,T );GFunction( 4,[XplotQuiver(i),YplotQuiver(j),ZplotQuiver(k)],coil_radius ,T);GFunction( 5,[XplotQuiver(i),YplotQuiver(j),ZplotQuiver(k)],coil_radius,T );GFunction( 6,[XplotQuiver(i),YplotQuiver(j),ZplotQuiver(k)],coil_radius,T )];       %#ok<*SAGROW>
%         end
%     end
% end
%END  of  compute 3D map of the magnetic field produced by each coil for one amp.
%/////////////////////////////////////////////////////////////////////////


%Update state of the robot
LastMeasuredPosition=P;
LastMeasuredTime=0;
LastPositionMeasureTime=0;
MeasuredPosition=P;
CalculatedVelocity=[0,0,0];
Integral=0;
VelocitySetpoint = [0,0,0];
OrienSetpoint = [0,0,0];

%/////////////3D plot initial environment and trajectory//////////////
%////////////////////////////////////////////////////////////////////////
set(gcf, 'Position', PlotPosition)
figure(1)
%calculate robot roll pitch and yaw with deriction vector
roll = Orien(1);
pitch =Orien(2);
yaw = Orien(3);
x = cos(yaw)*cos(pitch);
y = sin(yaw)*cos(pitch);
z = sin(pitch);

subplot(2,2,[1,3])
cla
plot3(Ztraj,Xtraj,Ytraj,'LineWidth',1)
set(gca,'YDir','normal')
set(gca,'ZDir','normal')
set(gca,'XDir','normal')
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
set(gcf, 'Position', PlotPosition)
hold on
%     quiver3(Ztraj(i),Xtraj(i),Ytraj(i),orientation(i,3)',orientation(i,1)',orientation(i,2)','color','r')
robot = stlread('robot.stl');
p1 = patch(robot,'EdgeColor','none','FaceColor',[0,0,0],'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.6);
R_X=[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)];
R_Y = [cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)];
R_Z=[cos(yaw-pi/2),-sin(yaw-pi/2),0;sin(yaw-pi/2),cos(yaw-pi/2),0;0,0,1];
p1.Vertices=(R_Z*R_X*R_Y*p1.Vertices')'+ P(1)*1000.*repmat([1,0,0],[size(robot,1),1])+P(2)*1000.*repmat([0,1,0],[size(robot,1),1])+P(3)*1000.*repmat([0,0,1],[size(robot,1),1]);
camlight
view([90,90,90])

drawnow

%         subplot(2,2,2)
%         for i=1:NbPtsX
%             for j=1:NbPtsY
%                 for k=1:NbPtsZ
%                     TempField=MagField3D([Xplot(i);Yplot(j);Zplot(k)],NbTurns.*I,coil_radius,T );
%                     NormB(i,j,k)=norm(TempField);
%                 end
%             end
%         end
%
%         slice(Xg,Yg,Zg,NormB,0,0,0)
%         title('Field Magnitude')
%         xlabel('x')
%         ylabel('y')
%         zlabel('z')

subplot(2,2,4)
cla
grid on

hold on
hRobOri = quiver3(0,0,0,5*x,5*y,5*z,'color','g');
hClosestPt = quiver3(0,0,0,10*x,10*y,10*z,'color','r');

p1v2 = patch(robot,'EdgeColor','none','FaceColor',[0,0,0],'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.6);
R_X=[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)];
R_Y = [cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)];
R_Z=[cos(yaw-pi/2),-sin(yaw-pi/2),0;sin(yaw-pi/2),cos(yaw-pi/2),0;0,0,1];
p1v2.Vertices=(R_Z*R_X*R_Y*p1v2.Vertices')'+ 0.*repmat([1,0,0],[size(robot,1),1])+0.*repmat([0,1,0],[size(robot,1),1])+0.*repmat([0,0,1],[size(robot,1),1]);
axis([-5,5,-5,5,-5,5])
camlight
view([90,90,90])
drawnow



%///////////////////////////////////////////////////////////////////////
%///////////Begining the main loop////////////////////////////////

while start ==1
    if   ~mod(PhysicsIterNumber,ControllerSpeedFactor) %The controller runs ControllerSpeedFactor time slower than the physics simulation. The controller outputs are updated every ControllerSpeedFactor iterations.
        
        %Simulate the measurement of the position using the camera
        PreviousMeasuredPosition=MeasuredPosition;
        [ MeasuredPosition,LastMeasuredTime ] = GaussianNoiseSensing( P, time, LastMeasuredTime,PreviousMeasuredPosition );
        
        %Update data used by the Kalman filter
        PreviousState=NewState;
        PreviousErrorCovMatrix=NewErrorCovMatrix;
        
        %Compute controller output
        [NewState, NewErrorCovMatrix, LastPositionMeasureTime, Velocity,Orien, MinimumDistanceIndex, Integral] = CompleteController1(PreviousMeasuredPosition, MeasuredPosition, time,deltaT,Traj, LastPositionMeasureTime,LastTime,PreviousState, PreviousErrorCovMatrix,15,Integral,orientation);
        
        %Update last iteration time
        LastTime=time;
    end
    %Compute the physics
    NewP = NewState(1:3)+Velocity';
    NewV = Velocity';
    P=NewP; %Update current position
    V=NewV; %Update current velocity
    
    time=time+deltaT; %update time
    
    %/////////////3D plot environment and trajectory//////////////
    %////////////////////////////////////////////////////////////////////////
    
    set(gcf, 'Position', PlotPosition)
    figure(1)
    %calculate robot roll pitch and yaw with deriction vector
    roll = 0;
    pitch =atan2(Orien(3),sqrt(Orien(2)^2+Orien(1)^2));
    yaw = atan2(Orien(2),Orien(1));
    %         x = Orien(1);
    %         y = Orien(2);
    %         z = Orien(3);
    x = Orien(1);
    y = Orien(2);
    z = Orien(3);
    
    
    subplot(2,2,[1,3])
    cla
    plot3(Ztraj,Xtraj,Ytraj,'LineWidth',1)
    set(gca,'YDir','normal')
    set(gca,'ZDir','normal')
    set(gca,'XDir','normal')
    xlabel('X [mm]')
    ylabel('Y [mm]')
    zlabel('Z [mm]')
    set(gcf, 'Position', PlotPosition)
    hold on
    plot3(Ztraj(MinimumDistanceIndex),Xtraj(MinimumDistanceIndex),Ytraj(MinimumDistanceIndex),'*r')
    plot3([P(1)*1000,Ztraj(MinimumDistanceIndex)],[P(2)*1000,Xtraj(MinimumDistanceIndex)],[P(3)*1000,Ytraj(MinimumDistanceIndex)],'r')
    quiver3(P(1)*1000,P(2)*1000,P(3)*1000,5*x,5*y,5*z,'color','g')
    
    %     quiver3(Ztraj(i),Xtraj(i),Ytraj(i),orientation(i,3)',orientation(i,1)',orientation(i,2)','color','r')
    
    p1 = patch(robot,'EdgeColor','none','FaceColor',[0,0,0],'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.6);
    R_X=[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)];
    R_Y = [cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)];
    R_Z=[cos(yaw-pi/2),-sin(yaw-pi/2),0;sin(yaw-pi/2),cos(yaw-pi/2),0;0,0,1];
    p1OrigVert = p1.Vertices';
    p1.Vertices=(R_Z*R_X*R_Y*p1OrigVert)'+ P(1)*1000.*repmat([1,0,0],[size(robot,1),1])+P(2)*1000.*repmat([0,1,0],[size(robot,1),1])+P(3)*1000.*repmat([0,0,1],[size(robot,1),1]);
   % camlight
    view([90,90,90])
    
    %drawnow
    
    %         subplot(2,2,2)
    %         for i=1:NbPtsX
    %             for j=1:NbPtsY
    %                 for k=1:NbPtsZ
    %                     TempField=MagField3D([Xplot(i);Yplot(j);Zplot(k)],NbTurns.*I,coil_radius,T );
    %                     NormB(i,j,k)=norm(TempField);
    %                 end
    %             end
    %         end
    %
    %         slice(Xg,Yg,Zg,NormB,0,0,0)
    %         title('Field Magnitude')
    %         xlabel('x')
    %         ylabel('y')
    %         zlabel('z')
    
    subplot(2,2,4)
  %  cla
%     grid on
%     set(gca,'YDir','normal')
%     set(gca,'ZDir','normal')
%     set(gca,'XDir','normal')
%     xlabel('X [mm]')
%     ylabel('Y [mm]')
%     zlabel('Z [mm]')
%     
    %         Current_direct = [P(1)*1000,P(2)*1000,P(3)*1000]-[Ztraj(MinimumDistanceIndex),Xtraj(MinimumDistanceIndex),Ytraj(MinimumDistanceIndex)];
    %         Current_direct = Current_direct./norm(Current_direct);
    
    hold on
    set(hRobOri, 'UData',5*x,'VData',5*y,'WData',5*z); %   quiver3(0,0,0,5*x,5*y,5*z,'color','g')
    %         quiver3(0,0,0,10*Current_direct(1),10*Current_direct(2),10*Current_direct(3),'color','r')
    set(hClosestPt, 'UData',10*orientation(MinimumDistanceIndex,3),'VData',10*orientation(MinimumDistanceIndex,1),'WData',10*orientation(MinimumDistanceIndex,2)); 
    %quiver3(0,0,0,10*orientation(MinimumDistanceIndex,3),10*orientation(MinimumDistanceIndex,1),10*orientation(MinimumDistanceIndex,2),'color','b')
    
    %p1 = patch(robot,'EdgeColor','none','FaceColor',[0,0,0],'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.6);
    R_X=[1,0,0;0,cos(pitch),-sin(pitch);0,sin(pitch),cos(pitch)];
    R_Y = [cos(roll),0,sin(roll);0,1,0;-sin(roll),0,cos(roll)];
    R_Z=[cos(yaw-pi/2),-sin(yaw-pi/2),0;sin(yaw-pi/2),cos(yaw-pi/2),0;0,0,1];
    p1v2.Vertices=(R_Z*R_X*R_Y*p1OrigVert)'+ 0.*repmat([1,0,0],[size(robot,1),1])+0.*repmat([0,1,0],[size(robot,1),1])+0.*repmat([0,0,1],[size(robot,1),1]);
    %axis([-5,5,-5,5,-5,5])
    %camlight
    view([90,90,90])
    drawnow
    
    %Update iteration counter
    PhysicsIterNumber=PhysicsIterNumber+1;
    PositionHistory(PhysicsIterNumber+1,:) = P(:);
    distance = MinimumDistance(P, Traj(:,1:3));
    DistanceHistory(PhysicsIterNumber+1) = distance;
    disp('Iteration number')
    disp(PhysicsIterNumber)
    if PhysicsIterNumber>20
        start=0;
    end
end

function [distance] = MinimumDistance (P, Traj)
[m,~]=size(Traj);
NbPtsInterp=1000;%Number of points used to interpolate the trajectory


XTrajectory=Traj(:,1);
YTrajectory=Traj(:,2);
ZTrajectory = Traj(:,3);

LowResIndex=(linspace(0,1,m));
HighResIndex=linspace(0,1,NbPtsInterp);

XHighResTrajectory = (interp1(LowResIndex,XTrajectory,HighResIndex,'spline'));
Xtraj = XHighResTrajectory - (max(XHighResTrajectory)+min(XHighResTrajectory))/2;

YHighResTrajectory = (interp1(LowResIndex,YTrajectory,HighResIndex,'spline'));
Ytraj = YHighResTrajectory - (max(YHighResTrajectory)+min(YHighResTrajectory))/2;

ZHighResTrajectory = (interp1(LowResIndex,ZTrajectory,HighResIndex,'spline'));
Ztraj = ZHighResTrajectory - (max(ZHighResTrajectory)+min(ZHighResTrajectory))/2;

RobotDistanceFromPath=sqrt((P(1)*1000-Ztraj).^2+(P(2)*1000-Xtraj).^2+(P(3)*1000-Ytraj).^2);
distance=min(RobotDistanceFromPath);
end












