function [ parameter ] = ParameterInitiation( InitialState,FinalState,LookUpTable )
%PARAMETERINITIATION Summary of this function goes here
%   Detailed explanation goes here

lookup_table=LookUpTable;
parameter=ones(4,1);
theta_step=pi/36;%0.0873
theta_max=10*pi/36;%0.8727
Arraylength=int32(2*theta_max/theta_step+1);
AngNumEachPosit=5;

%Calculate the position in the Vehicle-Coordinate
rotate_theta=InitialState(3);
x_temp=FinalState(1)-InitialState(1);
y_temp=FinalState(2)-InitialState(2);
FinalState(1)=x_temp*cos(rotate_theta)+y_temp*sin(rotate_theta);
FinalState(2)=-x_temp*sin(rotate_theta)+y_temp*cos(rotate_theta);
FinalState(3)=FinalState(3)-rotate_theta;

%Calculate the actual angle
theta=atan(FinalState(2)/FinalState(1));

if theta > theta_max
    parameter=lookup_table(1, 1).ParaMatrix(:,Arraylength);
else if theta < -theta_max
        parameter=lookup_table(1, 1).ParaMatrix(:,1);
    else
        A=-theta_max:theta_step:theta_max;
        B=abs(A'-(theta)*ones(Arraylength,1));
        index=find(B==min(B),1);
        parameter=lookup_table(1, 1).ParaMatrix(:,1+AngNumEachPosit*(index-1));
    end
end

end

