function [NewState, NewErrorCovMatrix , I,LastPositionMeasureTime,Orien, ActOut,MinimumDistanceIndex,Integral] = CompleteController1(PreviousMeasuredPosition, MeasuredPosition, time,deltaT,Traj, LastPositionMeasureTime,LastTime,RobotMass,Robot_diameter, DragCoef,IPrevious,PreviousState, PreviousErrorCovMatrix,SpeedOverwrite,ActIn,Integral,orientation,MagneticMoment,V)
%Overwrite the trajectory speed data.
Traj(:,4)=SpeedOverwrite.*Traj(:,4);
Rho = 1000; %[Kg/m^3]
T=0.31;%[m] length of the side of the cube formed by the coils.
coil_radius=0.1; %[m] Average redius of the electromagnets
NbTurns=795; %[no unit] Number of turns in the electromagnet
MaxI=1.4; %Maximum current in the coils (saturation value)

Kp=10; %Proportional component of trajectory pid
Ki=0e-3; %Proportional component of trajectory pidf
KpFD=0.3;%1;%.01;%;s1%1;%0.01;%e-2; %Proportional component of velocity pid
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
[ NewState, NewErrorCovMatrix] = KalmanFilter( PreviousState, PreviousErrorCovMatrix, MeasuredPosition, NewPositionMeasured,time, LastTime, DragCoef, RobotMass, NbTurns.*IPrevious, MeasuredVelocity,ActIn);
P=NewState(1:3);
%trajectory PID controller
[TotalForce,Orien,MinimumDistanceIndex,Integral] = TrajectoryController(P, Traj,orientation,Kp,Ki, MaxVelocity,RobotMass,Robot_diameter,DragCoef(1),Rho,Integral,deltaT);
if V(1) ==0 && V(2)==0 && V(3)==0
    [I,ActOut]=InverseMagnetics(TotalForce,Orien,P,coil_radius,MagneticMoment,T,MaxI,NbTurns,IPrevious,RobotMass,V);
else
    [I,ActOut]=InverseMagnetics(TotalForce,V./norm(V),P,coil_radius,MagneticMoment,T,MaxI,NbTurns,IPrevious,RobotMass,V);

end
disp('current Inverse')
disp(I')
end