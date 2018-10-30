function [ NewState, NewErrorCovMatrix] = KalmanFilter( PreviousState, PreviousErrorCovMatrix, LastMeasuredPosition, NewPositionMeasured,Time, LastTime, DragCoef, Mass, I, MeasuredVelocity,Act)
%/////////////////////////////////////////////////////////////////////////
%This program was made by Haoran, and Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 10/3/2018


dt=Time-LastTime; %calculate time step

%DragCoef is a coluunm vector containing [drag coefficient alog x axis;drag
%coefficient alog y axis; rotational drag coeficient]
%mass: mass of the robot [Kg]
%I: [A] current vector, i.e. control input

%NewPositionMeasured : boolean, true if a new position measurement was
%obtained

%Initialize Q and R covariance matrices
Q=4.*[1,0,0,0.5,0,0; 0,1,0,0,0.5,0;0,0,1,0,0,0.5;0.5,0,0,10,0,0; 0,0.5,0,0,10,0,;0,0,0.5,0,0,10];
R=eye(6,6).*0.001; %]eye(length(PreviousState));

%Build the A matrix of the Kalman Filter
A=[1 0 0 dt 0 0;0 1 0 0 dt 0;0 0 1 0 0 dt;0 0 0 1-dt.*DragCoef(1)./Mass 0 0;0 0 0 0 1-dt.*DragCoef(2)./Mass 0;0 0 0 0 0 1-dt.*DragCoef(3)./Mass];
%Build the B matrix of the Kalman Filter

B=[Act(1,:).*dt.*dt.*0.5./Mass;Act(2,:).*dt.*dt.*0.5./Mass;Act(3,:).*dt.*dt.*0.5./Mass;Act(1,:).*dt./Mass;Act(2,:).*dt./Mass;Act(3,:).*dt./Mass];

%////////////////////////////////////////////////////////////////////////
%Prediction Step
%newstate [Px, Py, Pz, Vx, Vy, Vz];
NewState=A*PreviousState+B*I;

NewErrorCovMatrix=A*PreviousErrorCovMatrix*A'+Q;


%Measurement update step

if NewPositionMeasured   
   
    Measurement=[LastMeasuredPosition(1);LastMeasuredPosition(2);LastMeasuredPosition(3);MeasuredVelocity(1);MeasuredVelocity(2);MeasuredVelocity(3)];
    C=eye(6,6);

    %Calculate Kalman Coeficient
    K=(NewErrorCovMatrix*C')/(C*NewErrorCovMatrix*C'+R);

    %Calculate a posteriori state
    NewState=NewState+K*(Measurement-C*NewState);

else
    NewState = PreviousState;
end

end