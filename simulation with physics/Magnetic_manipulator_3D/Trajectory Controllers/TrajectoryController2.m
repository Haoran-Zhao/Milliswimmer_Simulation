function [VelocityX,VelocityY,VelocityZ,SetpointRoll,SetpointPitch,SetpointYaw,MinimumDistanceIndex] = TrajectoryController2(FilteredPosition,Traj,orientation,KpXY,KpTheta, MaxVelocity)

%/////////////////////////////////////////////////////////////////////////
%This program was made by HaoranZhao, and Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 10/3/2018

%This function is the trajectory controller
%This function takes as input the position of the robot and the trajectoiry
%data. It calculates the velocity setpoint to be sent to the velocity
%controller.
P=FilteredPosition;
[m,~]=size(Traj);
%DistanceTargetPointResolution=0.0001; %[m] distance step to search the closest point of the trajectory
NbPtsInterp=1000;%Number of points used to interpolate the trajectory


XTrajectory=Traj(:,1);
YTrajectory=Traj(:,2);
ZTrajectory = Traj(:,3);
VelocityTrajectory=Traj(:,4);

% Roll=orientation(:,1);
% Pitch=orientation(:,2);
% Yall=orientation(:,3);


%Create high resolution trajectory via interpolation

%ExtraPoints=3; %positive integer This variable controls the number of points to add at the begining and the end of the trajectory data to make a smouth closed loop. Try to set to 0 to see its effect.  

%InitialLength=length(VelocityTrajectory);
% NegativeDistanceVelocityTrajectory=zeros(ExtraPoints,1);
% NegativeDistanceXTrajectory=zeros(ExtraPoints,1);
% NegativeDistanceYTrajectory=zeros(ExtraPoints,1);
% NegativeDistanceAngleTrajectory=zeros(ExtraPoints,1);
LowResIndex=(linspace(0,1,m));
HighResIndex=linspace(0,1,NbPtsInterp);

XHighResTrajectory = (interp1(LowResIndex,XTrajectory,HighResIndex,'spline'))-77.5;
YHighResTrajectory = (interp1(LowResIndex,YTrajectory,HighResIndex,'spline'));
ZHighResTrajectory = (interp1(LowResIndex,ZTrajectory,HighResIndex,'spline'))+77.5;


VelocityHighResTrajectory = (interp1(LowResIndex,VelocityTrajectory,HighResIndex,'spline'))';

%//////////////////////////////////////////////////////
%Sarch for the position of the trajectory that is the closest to the
%robot position. This position will be called TargetPoint. 
%DistanceTargetPoint is the distance that the robot would  needs to travel to reach TargetPoint from the first point on the trajectory if the control is perfect (the robot is always on the trajectory centerline). 
%The trajectory is devined using via points. An
%interpolation needs to be made between the via points. 
%Distance TargetPoint is used as an index to compute TargetPoint using the
%discretized path TrajectoryData.

RobotDistanceFromPath=sqrt((P(1)-ZHighResTrajectory).^2+(P(2)-XHighResTrajectory).^2+(P(3)-YHighResTrajectory).^2);
[~,MinimumDistanceIndex]=min(RobotDistanceFromPath);


% PathDistance=zeros(n,1);
% for i=2:n
%     PathDistance(i,1)=PathDistance(i-1)+sqrt((TrajectoryData(i,1)-TrajectoryData(i-1,1)).^2+(TrajectoryData(i,2)-TrajectoryData(i-1,2)).^2);
% end
% 
% 
% %Create high resolution trajectory via interpolation
% 
% ExtraPoints=3; %positive integer This variable controls the number of points to add at the begining and the end of the trajectory data to make a smouth closed loop. Try to set to 0 to see its effect.  
% 
% InitialLength=length(PathDistanceHighRes);
% for i=1:ExtraPoints
%     PathDistance(InitialLength+i)=PathDistance(InitialLength)+PathDistance(i);
%     NegativePathDistance(i)=-PathDistance(i);
% end
%     
% PathDistanceHighRes=linspace(0,PathDistance(n,1),round(PathDistance(n,1)/DistanceTargetPointResolution));
% XHighResTrajectory = (interp1(PathDistance,XTrajectory,PathDistanceHighRes,'spline'))';
% YHighResTrajectory = (interp1(PathDistance,YTrajectory,PathDistanceHighRes,'spline'))';
% AngleHighResTrajectory = (interp1(PathDistance,AngleTrajectory,PathDistanceHighRes,'spline'))';
% VelocityHighResTrajectory = (interp1(PathDistance,VelocityTrajectory,PathDistanceHighRes,'spline'))';
%     
% RobotDistanceFromPath=sqrt((P(1)-XHighResTrajectory(:,1)).^2+(P(2)-YHighResTrajectory(:,1)).^2);
% [NotUsed,MinimumDistanceIndex]=min( RobotDistanceFromPath);
% 
TargetPointX=ZHighResTrajectory(MinimumDistanceIndex);
TargetPointY=XHighResTrajectory(MinimumDistanceIndex);
TargetPointZ=YHighResTrajectory(MinimumDistanceIndex);

TargetVelocityNorm=VelocityHighResTrajectory(MinimumDistanceIndex);

if MinimumDistanceIndex==length(VelocityHighResTrajectory)
    TargetPointRoll = 0;
    TargetPointPitch =atan2(orientation(1,2),sqrt(orientation(1,3)^2+orientation(1,1)^2));
    TargetPointYaw = atan2(orientation(1,1),orientation(1,3));
    MinimumDistanceIndex =1;
else
    TargetPointRoll = 0;
    TargetPointPitch =atan2(orientation(MinimumDistanceIndex,2),sqrt(orientation(MinimumDistanceIndex,3)^2+orientation(MinimumDistanceIndex,1)^2));
    TargetPointYaw = atan2(orientation(MinimumDistanceIndex,1),orientation(MinimumDistanceIndex,3));
end
%calculate x y z velocity
TargetVelocityX=TargetVelocityNorm*orientation(MinimumDistanceIndex,1);
TargetVelocityY=TargetVelocityNorm*orientation(MinimumDistanceIndex,2);
TargetVelocityZ=TargetVelocityNorm*orientation(MinimumDistanceIndex,3);


ErrorPosX=TargetPointX-P(1);
ErrorPosY=TargetPointY-P(2);
ErrorPosZ=TargetPointZ-P(3);

VelocityXToCorrectPosition=KpXY.*ErrorPosX;
VelocityYToCorrectPosition=KpXY.*ErrorPosY;
VelocityZToCorrectPosition=KpXY.*ErrorPosZ;


%ErrorPosTheta=TargetPointTheta-P(3);
%VelocityThetaToCorrectPosition=KpTheta.*ErrorPosTheta;
% 
% if TargetPointRoll>(pi/2) && P(3)<(pi-TargetPointRoll)
%     VelocityThetaToCorrectPosition=KpTheta.*(2.*pi-TargetPointRoll+P(3));
% elseif TargetPointRoll<-(pi/2) && P(3)>(pi+TargetPointRoll)
%     VelocityThetaToCorrectPosition=KpTheta.*(2.*pi+TargetPointRoll-P(3));
% else
%     VelocityThetaToCorrectPosition=KpTheta.*(TargetPointRoll-P(3));
% end

RollToCorrectPosition=KpTheta.*(TargetPointRoll-P(4));
PitchToCorrectPosition=KpTheta.*(TargetPointPitch-P(5));
YawToCorrectPosition=KpTheta.*(TargetPointYaw-P(6));



VelocitySetpointTemp=[VelocityXToCorrectPosition+TargetVelocityX , VelocityYToCorrectPosition+TargetVelocityY, VelocityZToCorrectPosition+TargetVelocityZ];


if norm(VelocitySetpointTemp)>MaxVelocity
    VelocitySetpointTemp=MaxVelocity.*VelocitySetpointTemp./norm(VelocitySetpointTemp);
end
VelocityX=VelocitySetpointTemp(1);
VelocityY=VelocitySetpointTemp(2);
VelocityZ=VelocitySetpointTemp(3);

SetpointRoll=RollToCorrectPosition;
SetpointPitch=PitchToCorrectPosition;
SetpointYaw=YawToCorrectPosition;

% HistoryErrorPosXOutput=0;
% HistoryErrorPosYOutput=0;
% HistoryErrorPosThetaOutput=0;
