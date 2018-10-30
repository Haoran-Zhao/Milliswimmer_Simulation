%Traj=0.001.*[0,104.670000000000,0;0,109.700000000000,-1.80000000000000;0,115.500000000000,-9.40000000000001;0,115,-24.5000000000000;0,104,-39.5000000000000;0,91,-44.5000000000000;0,86.5000000000000,-44;1,77.5000000000000,-44;4,72.5000000000000,-44;10,70.5000000000000,-44;14,70.5000000000000,-44;25,70.5000000000000,-44;28.6350000000000,70.5000000000000,-42.3000000000000;31,70.5000000000000,-39;32,70.5000000000000,-31;32,70.5000000000000,-4;32,70.5000000000000,10;32.5000000000000,70.5000000000000,19.9900000000000;32.5000000000000,70.3000000000000,21.2000000000000;32.5000000000000,69.7000000000000,21.8000000000000;32.5000000000000,68,22;32.5000000000000,62.4000000000000,22.0200000000000;32.5000000000000,55.5800000000000,22.0110000000000;32.5000000000000,38.9300000000000,22.0110000000000;32.5000000000000,21.4140000000000,22.0110000000000;32.5000000000000,14.0550000000000,20;32.5000000000000,11.2500000000000,13.5000000000000;32.5450000000000,11,8.90000000000001;29,11,2;23,11,0;19,11,0;13,11,0;9,11.5000000000000,0;2.67200000000000,13.9600000000000,0;0,18.5000000000000,0;0,23,0;0,37.1500000000000,0;0,61.0400000000000,0;0,99.2000000000000,0;0,104.700000000000,0]
Traj=0.001.*[0,23,0;0,99.2000000000000,0;1,106.500000000000,0;5,111.500000000000,0;15.7180000000000,118.150000000000,0;35.0400000000000,120,-4;39,121.300000000000,-13;38.0100000000000,121.200000000000,-22;35,120.800000000000,-26;0,111.500000000000,-27.5000000000000;-40,101.500000000000,-27.5000000000000;-49.0200000000000,96.5000000000000,-35;-40,91.5000000000000,-42.5000000000000;0,81.5000000000000,-42.5000000000000;40,71.5000000000000,-42.5000000000000;49,66.5000000000000,-35;40,61.5000000000000,-27.5000000000000;0,51.5000000000000,-27.5000000000000;-40,41.5000000000000,-27.5000000000000;-49.0200000000000,36.5000000000000,-35;-40,31.5000000000000,-42.5000000000000;0,21.5000000000000,-42.5000000000000;40,11.5400000000000,-42.5000000000000;49.3300000000000,11.3100000000000,-38.4900000000000;49,10.6700000000000,-24.9700000000000;44,9.97000000000000,-9.98000000000000;31.5800000000000,9.57000000000000,-1.55000000000000;23,9.50000000000000,0;14,9.50000000000000,0;10.6500000000000,9.70000000000000,0;6.40000000000001,10.6900000000000,0;3.40000000000001,12.5700000000000,0;0.709999999999994,15.9000000000000,0;0,18.5000000000000,0;0,23,0]
%User inputs

MaxPlaneRotationSpeed=20; %[rad/s]



%Calculate a little circle to show the robot on the plots
XCircle=0;
YCircle=0;
CircleRadius=0.002;
AngleCircle=0;
i=1;
while true
    XCircleBot(i)=CircleRadius.*cos(AngleCircle);
    YCircleBot(i)=CircleRadius.*sin(AngleCircle);
    AngleCircle=AngleCircle+0.5;
    i=i+1
    if AngleCircle>2*pi
        break
    end
end
XCircleBot=XCircleBot+XPos;
ZCircleBot=YCircleBot+ZPos;
YCircleBot=YCircleBot+YPos;
%////////////////////////////////////////////////////////////////////


%////////////////////////////////////////////////////////////////////
%Calculate the coordinates of the points on a circle that will define the
%trajectory
%////////////////////////////////////////////////////////////////////
%XCircle=0;
%YCircle=0;
%CircleRadius=0.05;
%AngleCircle=0;
%i=1;
%while true
   % XCircle(i)=CircleRadius.*cos(AngleCircle);
  %  YCircle(i)=CircleRadius.*sin(AngleCircle);
   % AngleCircle=AngleCircle+0.5;
    %i=i+1
   % if AngleCircle>2*pi
      %  break
    %end
%end
%////////////////////////////////////////////////////////////////////



%///////////////////////////////////////////////////////////////////
%rotate the circle
%/////////////////////////////////////////////////////////////////
%Calculate rotation matrix
%AngleXTrajectory=0;

%AngleYTrajectory=0.1.*time;
%RotationMatrixX=[1 0 0;0 cos(-AngleXTrajectory) -sin(-AngleXTrajectory);0 sin(-AngleXTrajectory) cos(-AngleXTrajectory)];
%RotationMatrixY=[cos(-AngleYTrajectory) 0 sin(-AngleYTrajectory);0 1 0 ; -sin(-AngleYTrajectory) 0 cos(-AngleYTrajectory)];
%RotationMatrix=RotationMatrixX*RotationMatrixY;
%Trajectory=[];
%Make the rotation
%for k=1:length(XCircle)
 
%   Trajectory(:,k)=RotationMatrix*[XCircle(k);YCircle(k);0];
%end
%Trajectory=Trajectory';
%///////////////////////////////////////////////////////////////////////
% Tajectory [Z, X Y]
Trajectory=[-1.2.*Traj(:,3)+0.004,-0.7.*Traj(:,1)+0.0085,1.1.*Traj(:,2)-0.077];




%///////////////////////////////////////////////////////////////
%The last point of the trajectory is the first point
%//////////////////////////////////////////////////////////////
Trajectory(length(XCircle)+1,:)=Trajectory(1,:);
%//////////////////////////////////////////////////////////////





%//////////////////////////////////////////////////////////////////
%Calculate HD Trajectory
%/////////////////////////////////////////////////////////////////
[m,n]=size(Trajectory);
Index=linspace(0,m-1,m);
IndexHD=linspace(0,m-1,100);
%And here is the trajectory to follow!
XHD=interp1(Index,Trajectory(:,1),IndexHD,'spline');
YHD=interp1(Index,Trajectory(:,2),IndexHD,'spline');
ZHD=interp1(Index,Trajectory(:,3),IndexHD,'spline');
%////////////////////////////////////////////////////////////////

ReduceSpeed=false;
if XPos<0.02 && ZPos<-0.035 && YPos>-0.02 && YPos<0.02
ReduceSpeed=true; 
end


%/////////////////////////////////////////////////////////////////
%Trajectory Controller
%///////////////////////////////////////////////////////
%User inputs
Vopt=0.3; %Velocity setpoint [m/s]
if ReduceSpeed
Vopt=0.2;
end
%Search closest point on trajectory
RobotPosition=[XPos,YPos,ZPos];
for k=1:length(XHD)
    distance(k)=((RobotPosition(1)-XHD(k)).^2)+((RobotPosition(2)-YHD(k)).^2)+((RobotPosition(3)-ZHD(k)).^2)
end
[Min,IndexMin]=min(distance);
if IndexMin<length(XHD)
    %Do nothing
else
    IndexMin=1;
end
XCTP=XHD(IndexMin);
YCTP=YHD(IndexMin);
ZCTP=ZHD(IndexMin);
%Calculate the optimum velocity vector
TargetPoint=[XHD(IndexMin),YHD(IndexMin),ZHD(IndexMin)];
TargetPointDirection=([XHD(IndexMin+1),YHD(IndexMin+1),ZHD(IndexMin+1)]-TargetPoint)./norm(([XHD(IndexMin+1),YHD(IndexMin+1),ZHD(IndexMin+1)]-TargetPoint));
OptimumVelocity=Vopt.*TargetPointDirection;


%/////////////////////////////////////////////////////////////////
%Fluid mechanics analytical model
%///////////////////////////////////////////////////////
BotDiameter=0.005; %[m]
CD=0.47; %drag coeficient 
Rho=1000; %[Kg/m3]
Fd=CD.*Rho.*Vopt.*pi.*BotDiameter.*BotDiameter/4;
FdVect=Fd.*TargetPointDirection;


%PID Controller
Kp=0.11;
Kd=0e-6;
Ki=0e-3;
ErrorPosition=-[RobotPosition(1)-TargetPoint(1),RobotPosition(2)-TargetPoint(2),RobotPosition(3)-TargetPoint(3)];
Integral=Integral+ErrorPosition.*deltaT;
IntegralOut=Integral;
Derivative=(ErrorPosition-PreviousErrorPosition)./deltaT;
ErrorPositionOut=ErrorPosition;
FPID=Kp.*ErrorPosition+Ki.*Integral+Kd.*Derivative; % force asked by PID controller [N]


KpOrientation=50;
Kg=1;

%Calculate weight of the robot
Mass=0.0002
Weight=9.81.*Mass.*[0,0,1];

%Calculate total force to apply to the robot
TotalForce=FdVect+FPID+Weight;
ForceMagnitude=norm(TotalForce);



%Calculate direction of the Force to apply (used to calculate angles later)
DirectionSetpoint=TotalForce./norm(TotalForce);
%//////////////////////////////////////////////////////////////////////////////////////////////////




%/////////////////////////////////////////////////////////////////////////////////////
%Rotation speed controller
MaxOmega=725;
MinOmega=100;
OmegaIncrement=0.1; %[rad/s]
ThrustCoeficient=0.000004; %[N.s/rad]
OptimumRotationSpeed=norm(TotalForce)./ThrustCoeficient;
if OptimumRotationSpeed>Omega
         Omega=Omega+OmegaIncrement;
else
         Omega=Omega-OmegaIncrement;
end
if Omega>MaxOmega
Omega=MaxOmega;
end
if Omega<MinOmega
Omega=MinOmega;
end

%//////////////////////////////////////////////////////////////
%Calculate Circles to show the closest point on trajectory
%////////////////////////////////////////////////////////////
XCircleCTP=XCircleBot+XCTP-XPos;
YCircleCTP=YCircleBot+YCTP-YPos;
ZCircleCTP=ZCircleBot+ZCTP-ZPos;
%///////////////////////////////////////////////////////////////







%Generate line of Direction setpoint to plot
DirectionSetpointX=[0,0.1*DirectionSetpoint(1)];
DirectionSetpointY=[0,0.1*DirectionSetpoint(2)];
DirectionSetpointZ=[0,0.1*DirectionSetpoint(3)];

%/////////////////////////////////////////////////////////////////////////////////////////
%Calculate the two rotation angles
%/////////////////////////////////////////////////////////////////////////////////////////
%Find the values of the angles of direction Setpoint
AngleY=asin(DirectionSetpoint(1));
AngleX=acos(DirectionSetpoint(3)./cos(AngleY));

AngleX=sign(DirectionSetpoint(2)).*AngleX;
AngleY=-1.*AngleY;

MaxAngleStep=MaxPlaneRotationSpeed.*deltaT;

if ForceVertical
AngleX=0;
AngleY=0;
end

if AngleX-PreviousAngleX>MaxAngleStep
        AngleX=PreviousAngleX+MaxAngleStep
end
if AngleX-PreviousAngleX<-MaxAngleStep
        AngleX=PreviousAngleX-MaxAngleStep
end
if AngleY-PreviousAngleY>MaxAngleStep
        AngleY=PreviousAngleY+MaxAngleStep
end
if AngleY-PreviousAngleY<-MaxAngleStep
        AngleY=PreviousAngleY-MaxAngleStep
end




%Verify the calculation of angles
TestVector=[0;0;1];
RotationMatrixXTest=[1 0 0;0 cos(-AngleX) -sin(-AngleX);0 sin(-AngleX) cos(-AngleX)];
RotationMatrixYTest=[cos(-AngleY) 0 sin(-AngleY);0 1 0 ; -sin(-AngleY) 0 cos(-AngleY)];
RotationMatrixTest=RotationMatrixXTest*RotationMatrixYTest;
RotatedTest=RotationMatrixTest*TestVector;

DirectionSetpointX2=RotatedTest(1);
DirectionSetpointY2=RotatedTest(2);
DirectionSetpointZ2=RotatedTest(3);
%Verify the calculation of angles
%TestVector=[0;0;1];
%RotationMatrixX=[1 0 0;0 cos(-AngleX) -sin(-AngleX);0 sin(-AngleX) cos(-AngleX)];
%RotationMatrixY=[cos(-AngleY) 0 sin(-AngleY);0 1 0 ; -sin(-AngleY) 0 cos(-AngleY)];
%RotationMatrix=RotationMatrixX*RotationMatrixY;
%RotatedTest=RotationMatrix*TestVector;
%/////////////////////////////////////////////////////////////////////////////////////////




k=1; %coil constant [T/A] 


%CX=-15;
%CY=150;

%AngleX=CX.*YPos;
%AngleY=CY.*XPos;

%if AngleY>pi/2
%AngleY=pi/2;
%end

%if AngleY<-pi/2
%AngleY=-pi/2;
%end

%if AngleX>pi/2
%AngleX=pi/2;
%end

%if AngleX<-pi/2
%AngleX=-pi/2;
%end

%if ZPos>ZSetpoint
%OmegaOut=Omega-0.005;
%else
%OmegaOut=Omega+0.005;
%end

%if OmegaOut>160
%OmegaOut=160;
%end

%if OmegaOut<80
%OmegaOut=80;
%end
OmegaOut=Omega;

Bx2=NormB.*cos(RobotAngle);
By2=NormB.*cos(RobotAngle-pi/2);
    
%Calculate rotation matrix
RotationMatrixX=[1 0 0;0 cos(-AngleX) -sin(-AngleX);0 sin(-AngleX) cos(-AngleX)];
RotationMatrixY=[cos(-AngleY) 0 sin(-AngleY);0 1 0 ; -sin(-AngleY) 0 cos(-AngleY)];
RotationMatrix=RotationMatrixX*RotationMatrixY;
    
B=RotationMatrix*[Bx2;By2;0];

Ix=B(1)./k;
Iy=B(2)./k;
Iz=B(3)./k;

Saturation=false;

if Ix >MaxI
     Ix=MaxI;
     Saturation=true;
end

if Ix <-MaxI
     Ix=-MaxI;
     Saturation=true;
end

if Iy >MaxI
     Iy=MaxI;
     Saturation=true;
end

if Iy <-MaxI
     Iy=-MaxI;
     Saturation=true;
end

if Iz >MaxI
     Iz=MaxI;
     Saturation=true;
end

if Iz <-MaxI
     Iz=-MaxI;
     Saturation=true;
end

IxP=Ix;
IxN=-Ix;
IyP=Iy;
IyN=-Iy;
IzP=Iz;
IzN=-Iz;
