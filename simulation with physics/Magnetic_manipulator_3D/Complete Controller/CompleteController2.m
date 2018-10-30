function [P,NewState, NewErrorCovMatrix ,LastPositionMeasureTime, PreviousCorrectAngleMeasuredOut,MinimumDistanceIndex,VelocitySetpoint,OrienSetpoint] = CompleteController2(PreviousMeasuredPosition, MeasuredPosition, time,Traj,orientation,m, LastPositionMeasureTime,LastTime,Mass , DragCoef,PreviousState, PreviousErrorCovMatrix, J, IPrevious, PreviousCorrectAngleMeasuredIn,SpeedOverwrite,ActIn)
%Overwrite the trajectory speed data.
Traj(:,4)=SpeedOverwrite.*Traj(:,4);

T=0.31;%[m] length of the side of the cube formed by the coils.
coil_radius=0.1; %[m] Average redius of the electromagnets
NbTurns=795; %[no unit] Number of turns in the electromagnet
MaxI=1.4; %Maximum current in the coils (saturation value)

KpXYZ=0.6; %Proportional component of trajectory pid
KpTheta=0.01; %Proportional component of trajectory pidf
KpFD=0.003;%1;%.01;%;s1%1;%0.01;%e-2; %Proportional component of velocity pid
KpFQ=0.0003;
MaxVelocity=0.1; %[m/s]Maximum velocity setpoint

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
% kalman filter update NewState and covariance matric
[ NewState, NewErrorCovMatrix, PreviousCorrectAngleMeasuredOut ] = KalmanFilter( PreviousState, PreviousErrorCovMatrix, MeasuredPosition, NewPositionMeasured,time, LastTime, DragCoef, Mass, J, NbTurns.*IPrevious, MeasuredVelocity, PreviousCorrectAngleMeasuredIn,MeasureDuration,ActIn);
P=NewState(1:6);
%trajectory PID controller
[VelocitySetpoint,OrienSetpoint,MinimumDistanceIndex] = TrajectoryController(P,Traj,orientation,KpXYZ,KpTheta, MaxVelocity);
end