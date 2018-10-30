function [I,Act] = InverseMagnetics(TotalForce,Orien,P,coil_radius,MagneticMoment,T,MaxI,NbTurns,IPrevious)
%INVERSEMAGNETICS Summary of this function goes here

%This function takes as input the magnetic flux density and force on d axis
%and returen the current to apply

%INUTS: 
%FieldForce: 1x6 vector containg the 3 components of the flux density to apply and
%the 3 componenets of the force to apply [Bx;By;Bz;Fx;Fy;Fz]

%coil_radius: average radius of the coil [m]

%P: [m] Position of the robot [x,y,z]
%m: magnetic moment vector [mx;my;mz]
%T: [m] distance between oposit coils

%Outputs: 
FieldForce = zeros(3,1);
%I: [A] curremt vector
%Build the actuation matrix 

G1=GFunction(1,P,coil_radius,T);
G2=GFunction(2,P,coil_radius,T);
G3=GFunction(3,P,coil_radius,T);
G4=GFunction(4,P,coil_radius,T);
G5=GFunction(5,P,coil_radius,T);
G6=GFunction(6,P,coil_radius,T);



Gx=[G1(1) G2(1) G3(1) G4(1) G5(1) G6(1)];
Gy=[G1(2) G2(2) G3(2) G4(2) G5(2) G6(2)];
Gz=[G1(3) G2(3) G3(3) G4(3) G5(3) G6(3)];


dG1dx=dGFunction( 1,P,coil_radius,1,T );
dG1dy=dGFunction( 1,P,coil_radius,2,T );
dG1dz=dGFunction( 1,P,coil_radius,3,T );


dG2dx=dGFunction( 2,P,coil_radius,1,T );
dG2dy=dGFunction( 2,P,coil_radius,2,T );
dG2dz=dGFunction( 2,P,coil_radius,3,T );


dG3dx=dGFunction( 3,P,coil_radius,1,T );
dG3dy=dGFunction( 3,P,coil_radius,2,T );
dG3dz=dGFunction( 3,P,coil_radius,3,T );


dG4dx=dGFunction( 4,P,coil_radius,1,T );
dG4dy=dGFunction( 4,P,coil_radius,2,T );
dG4dz=dGFunction( 4,P,coil_radius,3,T );


dG5dx=dGFunction( 5,P,coil_radius,1,T );
dG5dy=dGFunction( 5,P,coil_radius,2,T );
dG5dz=dGFunction( 5,P,coil_radius,3,T );


dG6dx=dGFunction( 6,P,coil_radius,1,T );
dG6dy=dGFunction( 6,P,coil_radius,2,T );
dG6dz=dGFunction( 6,P,coil_radius,3,T );


dGx_dx= [dG1dx(1) dG2dx(1) dG3dx(1) dG4dx(1) dG5dx(1) dG6dx(1)];
dGx_dy= [dG1dy(1) dG2dy(1) dG3dy(1) dG4dy(1) dG5dy(1) dG6dy(1)];
dGx_dz= [dG1dz(1) dG2dz(1) dG3dz(1) dG4dz(1) dG5dz(1) dG6dz(1)];

dGy_dx= [dG1dx(2) dG2dx(2) dG3dx(2) dG4dx(2) dG5dx(2) dG6dx(2)];
dGy_dy= [dG1dy(2) dG2dy(2) dG3dy(2) dG4dy(2) dG5dy(2) dG6dy(2)];
dGy_dz= [dG1dz(2) dG2dz(2) dG3dz(2) dG4dz(2) dG5dz(2) dG6dz(2)];

dGz_dx= [dG1dx(3) dG2dx(3) dG3dx(3) dG4dx(3) dG5dx(3) dG6dx(3)];
dGz_dy= [dG1dy(3) dG2dy(3) dG3dy(3) dG4dy(3) dG5dy(3) dG6dy(3)];
dGz_dz= [dG1dz(3) dG2dz(3) dG3dz(3) dG4dz(3) dG5dz(3) dG6dz(3)];

%Place single components of magnetic moment is dedicated variables
mx=MagneticMoment*Orien(1);
my=MagneticMoment*Orien(2);
mz=MagneticMoment*Orien(3);

%Actuation Matrix
% Act=[Gx;Gy;Gz;(mx.*dGx_dx+my.*dGy_dx+mz.*dGz_dx);(mx.*dGx_dy+my.*dGy_dy+mz.*dGz_dy);(mx.*dGx_dz+my.*dGy_dz+mz.*dGz_dz)];
Act=[(mx.*dGx_dx+my.*dGy_dx+mz.*dGz_dx);(mx.*dGx_dy+my.*dGy_dy+mz.*dGz_dy);(mx.*dGx_dz+my.*dGy_dz+mz.*dGz_dz)];

%calculate FieldForce for Inversemagnetics
% FieldForce(1:3) = MagField3D(P,NbTurns.*IPrevious,coil_radius,T);
% FieldForce(4:6) = TotalForce';
FieldForce(1:3) = TotalForce';
disp ("Total force");
disp(TotalForce)
%Inverse the matrix and return the current
Solution=Act'/(Act*Act')*FieldForce;

I = Solution./NbTurns;
if max(abs(I))>MaxI
    I = MaxI*(I./max(abs(I)));
end
% while abs(I(1))>MaxI || abs(I(2))>MaxI || abs(I(3))>MaxI || abs(I(4))>MaxI || abs(I(5))>MaxI || abs(I(6))>MaxI
%     TotalForce=0.95.*TotalForce;
%     FieldForce(1:3) = TotalForce';
%     Solution=Act'/(Act*Act')*FieldForce;
%     I = Solution./NbTurns;   
% end
% disp ("actuation matrix_inverse")
% disp(Act)
end

