function [ TotalCost ] = ComputeTrajectoryCost(InitialState,ControlParameter,ReferenceMatrix,ObstacleInforMatrix)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%Co:proximity to obstacles;Cd:deviation from the center path;Cs:smoothness
%criterion
global l_width
CollisionDistThreshold=0.5;                        %unit:m, it is sum of ego vehicle radius and obstacle radius
distanceRefCan=0;maxdistanceRefCan=0;
k_sum=0;maxKCurve=0;
kD_sum=0;maxKDevia=0;

[x_temp,y_temp,k_temp,k_curve_deviation]=trajectory_curve_sparse_point(ControlParameter,InitialState);%sparse the path;
CandidatePath=[x_temp',y_temp'];
% plot(CandidatePath(:,1),CandidatePath(:,2),'b');hold on;
% plot(ReferenceMatrix(:,1),ReferenceMatrix(:,2),'g');
sampleN=size(x_temp,2);
referencepointnum=size(ReferenceMatrix,1);

for i=1:sampleN
    %%%%%%%%%%%%%%%Calculate Co%%%%%%%%%%%%%%%%%%%
    [distanceObsCan,CollisionFlag,effectivepoint]=ObstacleCostCalculate(ObstacleInforMatrix,CandidatePath(i,:),CollisionDistThreshold);
    if CollisionFlag==1 break;
    end
    
    %%%%%%%%%%%%%%%Calculate Cd%%%%%%%%%%%%%%%%%%%
    distancetemp2Matrix=bsxfun(@minus,ReferenceMatrix(:,1:2),CandidatePath(i,:));
    distancetemp2Column=sqrt((distancetemp2Matrix(:,1)).^2+(distancetemp2Matrix(:,2)).^2);
    distancetemp2=norm(distancetemp2Column,1)/referencepointnum;
    distanceRefCan=distanceRefCan+distancetemp2;
    
    %%%%%%%%%%%%%%%Calculate Cs%%%%%%%%%%%%%%%%%%%
    kD_sum=kD_sum+abs(k_curve_deviation(i));
    k_sum=k_sum+abs(k_temp(i));
end

%%%%%%%%%%%%%%%Calculate Co%%%%%%%%%%%%%%%%%%%
if CollisionFlag==1
    Co=Inf;
else if effectivepoint == 0
        Co=0;
    else
        Co=distanceObsCan/effectivepoint;
    end
end
%%%%%%%%%%%%%%%Calculate Cd%%%%%%%%%%%%%%%%%%%
Cd=distanceRefCan/(sampleN*l_width*2);
%%%%%%%%%%%%%%%Calculate Cs%%%%%%%%%%%%%%%%%%%
%Cs=(k_sum/(maxKCurve*sampleN)+kD_sum/(maxKDevia*sampleN))/2;
Cs=(k_sum+kD_sum)/sampleN*2;%TO-DO��set the maximum curvature

Cost=[Co,Cd,Cs];
W=[0.47,1,0.015];%[wo,wd,ws];
%%%%%%%Result Parameter Table%%%%%%%
%staightline-with-obstacles:[0.47,1,0.015]
%curveline-without-obstacles:[0,1,0];
%curveline-with-obstacles:[0.47,1,0.015]
%%%%%%%%%%%%%%%%%%%%%%%%%%%
TotalCost=Cost*W'/3;
%disp(Cost);
end

