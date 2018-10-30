function [ Fvalue ] = GFunction( coil_number,P,a ,T)

%Fvalue: Flux density vector produced by a current of 1A circulating in
%coil coil_number

%a: radius of the current loop [m]

%coil_number: Indicate which coil produces the magnetic field in this calculation
                %1 -> coil X-
                %2 -> coil X+
                %3 -> coil Y-
                %4 -> coil Y+
                %5 -> coil Z-
                %6 -> coil Z+

%The sign of the current in the coils is defined as follow: A positive
%current creates a flux density at the center of the coil that point inside
%the workspace. 

%Place the coordinates X,Y,Z of the calculation point into X Y Z variables
X=P(1);
Y=P(2);
Z=P(3);


%The equation used to calculate the magnetic field later in this function
%is defined in a cylindrical coordinate system linked to the coil beeing calculated.
%The cylindrical coordinate system is oriented along the main axis of the coil calculated.
%The calculation point is, in the cylindrical coordinate system, placed at
%a radius Rc and a height Zc.
%It is necessary to convert the point location from the manipulator coordinate system to the
%coil coordinate system

if (coil_number==1) %X-
    Rc=sqrt(Y.^2+Z.^2);
    Zc=X+T/2;
end

if (coil_number==2) %X+
    Rc=sqrt(Y.^2+Z.^2);
    Zc=-X+T/2;
end

if (coil_number==3) %Y-
    Rc=sqrt(X.^2+Z.^2);
    Zc=Y+T/2;
end

if (coil_number==4) %Y+
    Rc=sqrt(X.^2+Z.^2);
    Zc=-Y+T/2;
end

if (coil_number==5) %Z-
    Rc=sqrt(X.^2+Y.^2);
    Zc=Z+T/2;
end

if (coil_number==6)%Z+
    Rc=sqrt(X.^2+Y.^2);
    Zc=T/2-Z;
end

%This function computes the magnetic field produced by a single current
%loop in a cylindrical system. The calculation uses equations presented in
%:Simple Analytic Expressions for the Magnetic Field of a Circular Current
%Loop, Simpson, James C.; Lane, John E.; Immer, Christopher D.; Youngquist,
%Robert C, NASA/TM-2013-21791.



%B: Magnetic flux density vector (Br,Btheta,Bz) in the cylindrical coordinate system [T].

mu0=4*pi*1e-7; %vacuum permeability 

%R=M(1) %radius position of the calculation point
%Z=M(3) %Z position of the calculation points

%//////////////////////////////////////////////////////////
%////// Here starts the calculation of the magnetic field using the
%equations of the paper.
k=(4.*a.*Rc)./((a+Rc).*(a+Rc)+Zc.*Zc);
[I1,I2] = ellipke(k,10000000);
Brc=((mu0)./(2.*pi.*Rc)).*(Zc./(sqrt((a+Rc).*(a+Rc)+Zc.*Zc))).*(-I1+((a.*a+Rc.*Rc+Zc.*Zc)./((a-Rc).*(a-Rc)+Zc.*Zc)).*I2);
Bzc=((mu0)./(2.*pi)).*(1./(sqrt((a+Rc).*(a+Rc)+Zc.*Zc))).*(I1+((a.*a-Rc.*Rc-Zc.*Zc)./((a-Rc).*(a-Rc)+Zc.*Zc)).*I2);
%End of the calculation of the magnetic field using the equations of the paper.
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

%There is cases where the result is NAN
%    at a=r and z=0: This point has no physical sens since the considered
%    wire is of infinitly small diameter
%    at r=0: The equations for Br returns NAN but the result is 0

%The following fix this problem:
% if isnan(Br)
%     Br=0;
% end
% if isnan(Bz)
%     Bz=0;
% end

if Rc==0
    Brc=0;
end

if abs(a-Rc)<(a/100) && abs(Zc)<(a/100)
    Brc=0;
    Bzc=0;
end



%The results Bz and Br are expressed in the electromagnet coordinate system. It need to be expressed in the workspace coordinate system X,Y,Z. 

if coil_number==1
   Bx=Bzc;
   if (Y^2+Z^2)==0
       By=0;
       Bz=0;
   else
        By=Brc.*(Y/(sqrt(Y.*Y+Z.*Z)));
        Bz=Brc.*(Y/(sqrt(Y.*Y+Z.*Z)));
   end
end


if coil_number==2
   Bx=-Bzc;
   if (Y^2+Z^2)==0
       By=0;
       Bz=0;
   else
        By=Brc.*(Y/(sqrt(Y.*Y+Z.*Z)));
        Bz=Brc.*(Y/(sqrt(Y.*Y+Z.*Z)));
   end
end

if coil_number==3
   By=Bzc;
   if (X^2+Z^2)==0
       Bx=0;
       Bz=0;
   else
        Bx=Brc.*(X/(sqrt(X.*X+Z.*Z)));
        Bz=Brc.*(Z/(sqrt(X.*X+Z.*Z)));
   end
end

if coil_number==4
   By=-Bzc;
   if (X^2+Z^2)==0
       Bx=0;
       Bz=0;
   else
        Bx=Brc.*(X/(sqrt(X.*X+Z.*Z)));
        Bz=Brc.*(Z/(sqrt(X.*X+Z.*Z)));
   end
end

if coil_number==5
   Bz=Bzc;
   if (X^2+Y^2)==0
       Bx=0;
       By=0;
   else
        Bx=Brc.*(X/(sqrt(X.*X+Y.*Y)));
        By=Brc.*(Y/(sqrt(X.*X+Y.*Y)));
   end
end

if coil_number==6
   Bz=Bzc;
   if (X^2+Y^2)==0
       Bx=0;
       By=0;
   else
        Bx=Brc.*(X/(sqrt(X.*X+Y.*Y)));
        By=Brc.*(Y/(sqrt(X.*X+Y.*Y)));
   end
end

Fvalue=[Bx;By;Bz]; %Flux densityper amp for each axis

end

