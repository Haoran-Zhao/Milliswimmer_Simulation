function [NewState, NewErrorCovMatrix,LastPositionMeasureTime,Velocity,Orien, MinimumDistanceIndex,Integral] = CompleteController1(PreviousMeasuredPosition, MeasuredPosition, time,deltaT,Traj, LastPositionMeasureTime,LastTime,PreviousState, PreviousErrorCovMatrix,SpeedOverwrite,Integral,orientation)
%Overwrite the trajectory speed data.
Traj(:,4)=SpeedOverwrite.*Traj(:,4);
Kp=0.5; %Proportional component of trajectory pid
Ki=0e-3; %Proportional component of trajectory pidf
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
[ NewState, NewErrorCovMatrix] = KalmanFilter( PreviousState, PreviousErrorCovMatrix, MeasuredPosition, NewPositionMeasured,time, LastTime, MeasuredVelocity);
P=NewState(1:3);
%trajectory PID controller
[Velocity,Orien,MinimumDistanceIndex,Integral] = TrajectoryController(P, Traj,Kp,Ki, MaxVelocity,Integral,deltaT,orientation);
end