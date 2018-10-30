function [Velocity,Orien,MinimumDistanceIndex,IntegralOut] = TrajectoryController(P, Traj,Kp,Ki, MaxVelocity,Integral,deltaT,orientation)
ratio = 0.1;
[m,~]=size(Traj);
NbPtsInterp=1000;%Number of points used to interpolate the trajectory


XTrajectory=Traj(:,1);
YTrajectory=Traj(:,2);
ZTrajectory = Traj(:,3);
VelocityTrajectory=Traj(:,4);

LowResIndex=(linspace(0,1,m));
HighResIndex=linspace(0,1,NbPtsInterp);

XHighResTrajectory = (interp1(LowResIndex,XTrajectory,HighResIndex,'spline'));
Xtraj = XHighResTrajectory - (max(XHighResTrajectory)+min(XHighResTrajectory))/2;

YHighResTrajectory = (interp1(LowResIndex,YTrajectory,HighResIndex,'spline'));
Ytraj = YHighResTrajectory - (max(YHighResTrajectory)+min(YHighResTrajectory))/2;

ZHighResTrajectory = (interp1(LowResIndex,ZTrajectory,HighResIndex,'spline'));
Ztraj = ZHighResTrajectory - (max(ZHighResTrajectory)+min(ZHighResTrajectory))/2;


VelocityHighResTrajectory = (interp1(LowResIndex,VelocityTrajectory,HighResIndex,'spline'))';

%find the closest point
RobotDistanceFromPath=sqrt((P(1)*1000-Ztraj).^2+(P(2)*1000-Xtraj).^2+(P(3)*1000-Ytraj).^2);
[~,MinimumDistanceIndex]=min(RobotDistanceFromPath);


while RobotDistanceFromPath(MinimumDistanceIndex)<5
    disp('MinimumDistanceIndex')
    disp(MinimumDistanceIndex)
    disp('Min Distance')
    disp(RobotDistanceFromPath(MinimumDistanceIndex))
    if MinimumDistanceIndex < 1000
        MinimumDistanceIndex = MinimumDistanceIndex+1;
    if MinimumDistanceIndex >=1000
        MinimumDistanceIndex=MinimumDistanceIndex-length(VelocityHighResTrajectory)+1;
    end
    end
    
    
end

disp('MinimumDistanceIndex')
disp(MinimumDistanceIndex)
disp('Min distance')
disp(RobotDistanceFromPath(MinimumDistanceIndex))

if (MinimumDistanceIndex)>length(VelocityHighResTrajectory)
    TargetPointX=Ztraj(MinimumDistanceIndex-length(VelocityHighResTrajectory))/1000;
    TargetPointY=Xtraj(MinimumDistanceIndex-length(VelocityHighResTrajectory))/1000;
    TargetPointZ=Ytraj(MinimumDistanceIndex-length(VelocityHighResTrajectory))/1000;
    TargetPointu=orientation(MinimumDistanceIndex-length(VelocityHighResTrajectory),1);
    TargetPointv=orientation(MinimumDistanceIndex-length(VelocityHighResTrajectory),2);
    TargetPointw=orientation(MinimumDistanceIndex-length(VelocityHighResTrajectory),3);
    TargetVelocityNorm=VelocityHighResTrajectory(MinimumDistanceIndex-length(VelocityHighResTrajectory));
    MinimumDistanceIndex =MinimumDistanceIndex-length(VelocityHighResTrajectory);
else
    TargetPointX=Ztraj(MinimumDistanceIndex)/1000;
    TargetPointY=Xtraj(MinimumDistanceIndex)/1000;
    TargetPointZ=Ytraj(MinimumDistanceIndex)/1000;
    TargetPointu=orientation(MinimumDistanceIndex,1);
    TargetPointv=orientation(MinimumDistanceIndex,2);
    TargetPointw=orientation(MinimumDistanceIndex,3);
    TargetVelocityNorm=VelocityHighResTrajectory(MinimumDistanceIndex);
end
disp("Current position")
disp(P')
disp("Target coordinate")
disp([TargetPointX,TargetPointY,TargetPointZ])
TargetPointVector = -[P(1)*1000,P(2)*1000,P(3)*1000]+[Ztraj(MinimumDistanceIndex),Xtraj(MinimumDistanceIndex),Ytraj(MinimumDistanceIndex)];
TargetPointDierection = TargetPointVector ./ norm(TargetPointVector);
% TargetVelocityNorm=norm(-[P(1)*1000,P(2)*1000,P(3)*1000]+[Ztraj(MinimumDistanceIndex),Xtraj(MinimumDistanceIndex),Ytraj(MinimumDistanceIndex)]);
%calculate x y z velocity
% TargetVelocityX=TargetVelocityNorm*TargetPointu;
% TargetVelocityY=TargetVelocityNorm*TargetPointv;
% TargetVelocityZ=TargetVelocityNorm*TargetPointw;
% OptimumVelocity = [TargetVelocityX,TargetVelocityY,TargetVelocityZ];
% Fd=CdForce.*Rho.*MaxVelocity.*pi.*Robot_diameter.*Robot_diameter/4;
% FdTargetVec = Fd.*TargetPointDierection;

%PID Controller
ErrorPosition=-[P(1)-TargetPointX,P(2)-TargetPointY,P(3)-TargetPointZ];
Integral=Integral+ErrorPosition.*deltaT;
IntegralOut=Integral;
VPID=Kp.*ErrorPosition+Ki.*Integral; % force asked by PID controller [N]
if norm(VPID) > MaxVelocity
%     point to the closed or ahead point
    VPID = MaxVelocity.*(VPID ./norm(VPID));
elseif norm(VPID) < ratio*MaxVelocity
    VPID = ratio*MaxVelocity.*(VPID ./norm(VPID));
end
Velocity = VPID;
%Calculate orientation
Orien = TargetPointDierection;
% disp("Target Orien")
% disp([TargetPointu,TargetPointv,TargetPointw])
disp('Orien')
disp(Orien)

end