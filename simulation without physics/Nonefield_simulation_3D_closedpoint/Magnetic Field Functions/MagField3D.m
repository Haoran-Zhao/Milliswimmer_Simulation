function [ B ] = MagField3D(P,I,a,T )
%MAGFIELD3D computes the magnetic field produced by the magnetic
%manipulator in 3D

%P: Calculation point [X;Y;Z]
%I: Vector containing the current of each coil [I1;I2;I3;I4;I5;I6]
%a: Average radius of the coils
%T: diatsnace between the coils

%B: Flux density vector [Bx;By;Bz]

%Calculate the flux density produced by coil 1
B1=I(1).*GFunction( 1,P,a ,T);

%Calculate the flux density produced by coil 2
B2=I(2).*GFunction( 2,P,a ,T);

%Calculate the flux density produced by coil 3
B3=I(3).*GFunction( 3,P,a ,T);

%Calculate the flux density produced by coil 4
B4=I(4).*GFunction( 4,P,a ,T);

%Calculate the flux density produced by coil 5
B5=I(5).*GFunction( 5,P,a ,T);

%Calculate the flux density produced by coil 6
B6=I(6).*GFunction( 6,P,a ,T);

%Calculate the total flux density
B=B1+B2+B3+B4+B5+B6;
end

