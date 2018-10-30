function [ NewP, NewV ] = ComputePhysics1( P,Orien,V, I , RobotMass,deltaT ,coil_radius,T,Cd,NbTurns,MagneticMoment,maxI)
%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018
% MaxVelocity=0.1; %[m/s]Maximum velocity setpoint
% ratio=0.1;
X=P(1);
Y=P(2);
Z=P(3);
if V(1) ==0 && V(2)==0 && V(3)==0
    u=Orien(1);
    v=Orien(2);
    w=Orien(3);
else
    u=V(1)/norm(V);
    v=V(2)/norm(V);
    w=V(3)/norm(V);
end
    
Vx=V(1);
Vy=V(2);
Vz=V(3);
% %Simulate saturation of currents
% for i=1:6
%     disp(I(i))
%     if I(i)>maxI
%         I(i)=maxI;
%     elseif I(i)<-maxI
%         I(i)=-maxI;
%     elseif 0<I(i)<0.1*maxI
%         I(i) = 0.1*maxI;
%     elseif -0.1*maxI<I(i)<0
%         I(i)=-0.1*maxI;
%     end
% end
    
disp('current physics')
disp(I')
%Perform the forward magnetic calculation
I=NbTurns.*I;

mx=MagneticMoment*u;
my=MagneticMoment*v;
mz=MagneticMoment*w;

%Here begins the calculation of the torque
Robot_Position=[X,Y,Z];
% G1=GFunction(1,Robot_Position,coil_radius,T);
% G2=GFunction(2,Robot_Position,coil_radius,T);
% G3=GFunction(3,Robot_Position,coil_radius,T);
% G4=GFunction(4,Robot_Position,coil_radius,T);
% G5=GFunction(5,Robot_Position,coil_radius,T);
% G6=GFunction(6,Robot_Position,coil_radius,T);
% 
% 
% Gx=[G1(1) G2(1) G3(1) G4(1) G5(1) G6(1)];
% Gy=[G1(2) G2(2) G3(2) G4(2) G5(2) G5(2)];
% Gz=[G1(3) G2(3) G3(3) G4(3) G5(3) G6(3)];

%Here begins the calculation of the force
dG1dx=dGFunction( 1,Robot_Position,coil_radius,1,T );
dG1dy=dGFunction( 1,Robot_Position,coil_radius,2,T );
dG1dz=dGFunction( 1,Robot_Position,coil_radius,3,T );


dG2dx=dGFunction( 2,Robot_Position,coil_radius,1,T );
dG2dy=dGFunction( 2,Robot_Position,coil_radius,2,T );
dG2dz=dGFunction( 2,Robot_Position,coil_radius,3,T );


dG3dx=dGFunction( 3,Robot_Position,coil_radius,1,T );
dG3dy=dGFunction( 3,Robot_Position,coil_radius,2,T );
dG3dz=dGFunction( 3,Robot_Position,coil_radius,3,T );


dG4dx=dGFunction( 4,Robot_Position,coil_radius,1,T );
dG4dy=dGFunction( 4,Robot_Position,coil_radius,2,T );
dG4dz=dGFunction( 4,Robot_Position,coil_radius,3,T );

dG5dx=dGFunction( 5,Robot_Position,coil_radius,1,T );
dG5dy=dGFunction( 5,Robot_Position,coil_radius,2,T );
dG5dz=dGFunction( 5,Robot_Position,coil_radius,3,T );

dG6dx=dGFunction( 6,Robot_Position,coil_radius,1,T );
dG6dy=dGFunction( 6,Robot_Position,coil_radius,2,T );
dG6dz=dGFunction( 6,Robot_Position,coil_radius,3,T );

dGx_dx= [dG1dx(1) dG2dx(1) dG3dx(1) dG4dx(1) dG5dx(1) dG6dx(1)];
dGx_dy= [dG1dy(1) dG2dy(1) dG3dy(1) dG4dy(1) dG5dy(1) dG6dy(1)];
dGx_dz= [dG1dz(1) dG2dz(1) dG3dz(1) dG4dz(1) dG5dz(1) dG6dz(1)];

dGy_dx= [dG1dx(2) dG2dx(2) dG3dx(2) dG4dx(2) dG5dx(2) dG6dx(2)];
dGy_dy= [dG1dy(2) dG2dy(2) dG3dy(2) dG4dy(2) dG5dy(2) dG6dy(2)];
dGy_dz= [dG1dz(2) dG2dz(2) dG3dz(2) dG4dz(2) dG5dz(2) dG6dz(2)];

dGz_dx= [dG1dx(3) dG2dx(3) dG3dx(3) dG4dx(3) dG5dx(3) dG6dx(3)];
dGz_dy= [dG1dy(3) dG2dy(3) dG3dy(3) dG4dy(3) dG5dy(3) dG6dy(3)];
dGz_dz= [dG1dz(3) dG2dz(3) dG3dz(3) dG4dz(3) dG5dz(3) dG6dz(3)];

Act = [mx.*dGx_dx+my.*dGy_dx+mz.*dGz_dx;mx.*dGx_dy+my.*dGy_dy+mz.*dGz_dy;mx.*dGx_dz+my.*dGy_dz+mz.*dGz_dz];
% disp("Actuation matrix_physics")
% disp(Act)

%////////////////////////////////////////////////////////////
F=Act*I;
disp("calculated force")
disp(F')
disp("To close point")
disp(Orien)
FrictionX=-Cd.*Vx;
FrictionY=-Cd.*Vy;
FrictionZ=-Cd.*Vz;


accelerationX=(F(1)+FrictionX)/RobotMass;
accelerationY=(F(2)+FrictionY)/RobotMass;
accelerationZ=(F(3)+FrictionZ)/RobotMass;
% accelerationX=F(1)/RobotMass;
% accelerationY=F(2)/RobotMass;
% accelerationZ=F(3)/RobotMass;

NewVelocityX=Vx+accelerationX.*deltaT;
NewVelocityY=Vy+accelerationY.*deltaT;
NewVelocityZ=Vz+accelerationZ.*deltaT;
% disp('New Velocity and acceleration')
% disp([NewVelocityX,NewVelocityY,NewVelocityZ])
% disp([accelerationX,accelerationY,accelerationZ])

% disp([NewVelocityX.*deltaT+0.5.*accelerationX.*deltaT.*deltaT;NewVelocityY.*deltaT+0.5.*accelerationY.*deltaT.*deltaT;NewVelocityZ.*deltaT+0.5.*accelerationZ.*deltaT.*deltaT]')

NewPositionX=X+NewVelocityX.*deltaT+0.5.*accelerationX.*deltaT.*deltaT;
NewPositionY=Y+NewVelocityY.*deltaT+0.5.*accelerationY.*deltaT.*deltaT;
NewPositionZ=Z+NewVelocityZ.*deltaT+0.5.*accelerationZ.*deltaT.*deltaT;


%Process Noise
% VarianceOfNoiseVelocity=0.00025;% [m/s]
% 
% NoiseVelocityPosition = VarianceOfNoiseVelocity.*randn(1,3);

NewP=[NewPositionX,NewPositionY,NewPositionZ];
% NewV=[NewVelocityX+NoiseVelocityPosition(1),NewVelocityY+NoiseVelocityPosition(2),NewVelocityZ+NoiseVelocityPosition(3)];
NewV=[NewVelocityX,NewVelocityY,NewVelocityZ];
% 
% disp('new position')
% disp (NewP)
disp('new velocity')
disp (NewV)
end

