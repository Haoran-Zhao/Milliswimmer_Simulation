function [NewState, NewErrorCovMatrix , I,LastPositionMeasureTime, PreviousCorrectAngleMeasuredOut, ActOut,MinimumDistanceIndex] = CompleteController(PreviousMeasuredPosition, MeasuredPosition, time,Traj,orientation,m, LastPositionMeasureTime,LastTime,Mass , DragCoef,PreviousState, PreviousErrorCovMatrix, J, IPrevious, PreviousCorrectAngleMeasuredIn,SpeedOverwrite,ActIn)

%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%This function is the complete controller composed of the filter, the
%trajectory controller, thr velocity controller and the inverse
%calculeation

%It takes as input the Position of the robot, the trajectory data and the previous sate data.
Traj(:,4)=SpeedOverwrite.*Traj(:,4);

T=0.31;%[m] length of the side of the cube formed by the coils.
coil_radius=0.1; %[m] Average redius of the electromagnets
NbTurns=795; %[no unit] Number of turns in the electromagnet
MaxI=1.4; %Maximum current in the coils (saturation value)

KpXYZ=1; %Proportional component of trajectory pid
% KdXYZ=0; %Derivative component of trajectory pid [not implemented, this constant is not used yet]
% KiXYZ=0; %Integral component of trajectory pid [not implemented, this constant is not used yet]
KpTheta=1; %Proportional component of trajectory pidf
% KdTheta=0; %Derivative component of trajectory pid [not implemented, this constant is not used yet]
% KiTheta=0; %Integral component of trajectory pid [not implemented, this constant is not used yet]
KpFD=0.003;%1;%.01;%;s1%1;%0.01;%e-2; %Proportional component of velocity pid
KpFQ=0.0003;
% KdF=0.001; %Derivative component of velocity pid [not implemented, this constant is not used yet]
% KiF=3e-3; %Integral component of velocity pid
% KpT=1.0e-3; %Proportional component of velocity pid
% KdT=0; %Derivative component of velocity pid [not implemented, this constant is not used yet]
% KiT=0e-4; %Integral component of velocity pid
MaxVelocity=1; %[m/s]Maximum velocity setpoint


if PreviousMeasuredPosition==MeasuredPosition
    NewPositionMeasured=false;
    MeasuredVelocity=0;
    
    MeasureDuration=0; %This variable needs to be assigned but is not used since no measurement eas acquired
else
    NewPositionMeasured=true;
    MeasureDuration=time-LastPositionMeasureTime;
    LastPositionMeasureTime=time;
    MeasuredVelocity=(MeasuredPosition-PreviousMeasuredPosition)./MeasureDuration;
end
    

%[ FilteredPosition , CalculatedVelocity ]=Filter2( [MeasuredPosition], PreviousFilteredPosition,PreviousCalculatedVelocity, NewPositionMeasured, MeasureDuration )   ;
 
[ NewState, NewErrorCovMatrix, PreviousCorrectAngleMeasuredOut ] = KalmanFilter( PreviousState, PreviousErrorCovMatrix, MeasuredPosition, NewPositionMeasured,time, LastTime, DragCoef, Mass, J, NbTurns.*IPrevious, MeasuredVelocity, PreviousCorrectAngleMeasuredIn,MeasureDuration,ActIn);


P=NewState(1:6);
% CalculatedVelocity=NewState(7:12);

[VelocitySetpoint,OrienSetpoint,MinimumDistanceIndex] = TrajectoryController(P,Traj,orientation,KpXYZ,KpTheta, MaxVelocity);
% HistoryErrorAngleVelocity=0;

%Perform the transform to the DQ coordinate system
%It is a rotation by an angle equal to the angle of the robot;

% %build rotation matrix
R_X=[1,0,0;0,cos(OrienSetpoint(2)),-sin(OrienSetpoint(2));0,sin(OrienSetpoint(2)),cos(OrienSetpoint(2))];
R_Y = [cos(OrienSetpoint(1)),0,sin(OrienSetpoint(1));0,1,0;-sin(OrienSetpoint(1)),0,cos(OrienSetpoint(1))];
R_Z=[cos(OrienSetpoint(3)-pi/2),-sin(OrienSetpoint(3)-pi/2),0;sin(OrienSetpoint(3)-pi/2),cos(OrienSetpoint(3)-pi/2),0;0,0,1];
RotationMatrix=R_Z*R_X*R_Y;

%Rotate the velocity setpoint
VsetpointDQ=RotationMatrix*VelocitySetpoint';

%Rotate the current state of the robot
%Remark: the rotated state has only 2 components as the angle is always
%zero in the rotated reference frame

%[NewState(1);NewState(2)]
% NewStateDQ=RotationMatrix*[NewState(7);NewState(8);NewState(9)];
NewStateDQ=[NewState(7);NewState(8);NewState(9)];


%[ ForceSetpoint , TorqueSetpoint, HistoryErrorVelocity , HistoryErrorAngleVelocity ] = VelocityControllerForDQInverse( CalculatedVelocity, [VelocitySetpointX,VelocitySetpointY] , VelocitySetpointTheta , KpF,KiF,KdF, KpT,KiT,KdT ,HistoryErrorVelocity, HistoryErrorAngleVelocity,  time, CdForceModel, CdTorqueModel );
[ FDQ , TorqueSetpoint] = VelocityControllerDQ( NewStateDQ, VsetpointDQ,  KpFD,KpFQ);%

%rotate the force setpoint to the original axis frame
% ForceSetpoint=RotationMatrix*FDQ;
ForceSetpoint=FDQ;

%pause(1)

[I,ActOut]=InverseMagnetics([P(1),P(2),P(3)],[P(4),P(5),P(6)],TorqueSetpoint, ForceSetpoint ,coil_radius,NbTurns,m,T,MaxI);
% 
% HistoryErrorVelocityOut=HistoryErrorVelocity;%,HistoryErrorAngleVelocity];
% HistoryErrorPositionOut=0;%[HistoryErrorPosXOutput,HistoryErrorPosYOutput,HistoryErrorPosThetaOutput];
% VelocitySetpoint=[VelocitySetpointX, VelocitySetpointY,VelocitySetpointZ,VelocitySetpointRoll,VelocitySetpointPitch,VelocitySetpointYaw];

end

