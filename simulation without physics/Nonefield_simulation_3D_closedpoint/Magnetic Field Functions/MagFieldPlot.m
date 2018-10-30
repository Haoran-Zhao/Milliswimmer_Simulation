clc
clear all
close all

%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%User Inputs
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
T=0.31;%[m] length of the side of the cube formed by the coils.
a=0.1; %[m] Average radius of the electromagnets
NbTurns=795; %[no unit] Number of turns in the electromagnet

I1=0; %Current in coil 1 [A] X-
I2=0; %Current in coil 2 [A] X+
I3=10; %Current in coil 3 [A] Y-
I4=0; %Current in coil 4 [A] Y+
I5=0; %Current in coil 5 [A] Z-
I6=0; %Current in coil 6 [A] Z+

Xmin=-0.15; %Min x value to plot
Xmax=0.15;  %Max x value to plot
NbPtsX=50; %Number of points to plot on x axis

Ymin=-0.15; %Min y value to plot
Ymax=0.15;  %Max y value to plot
NbPtsY=50; %Number of points to plot on y axis

Zmin=-0.15; %Min z value to plot
Zmax=0.15;  %Max z value to plot
NbPtsZ=50; %Number of points to plot on z axis
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%Variables definition
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
X=linspace(Xmin,Xmax,NbPtsX);
Y=linspace(Ymin,Ymax,NbPtsY);
Z=linspace(Zmin,Zmax,NbPtsZ);
[Xg,Yg,Zg]=meshgrid(X,Y,Z);

NormB=zeros(NbPtsX,NbPtsY,NbPtsZ);
I=[I1;I2;I3;I4;I5;I6];
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%Calculate the magnetic field
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
for i=1:NbPtsX
    for j=1:NbPtsY
        for k=1:NbPtsZ
            
            TempField=MagField3D([X(i);Y(j);Z(k)],NbTurns.*I,a,T );
            NormB(j,i,k)=norm(TempField);
        end
    end
end
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%Plot the results
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
%subplot(2,2,1)
slice(Xg,Yg,Zg,NormB,0,0,0)
title('Field Magnitude')
xlabel('x')
ylabel('y')
zlabel('z')
