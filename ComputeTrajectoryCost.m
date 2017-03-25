function [ TotalCost ] = ComputeTrajectoryCost(InitialState,ControlParameter,VelocityProfile,ReferenceMatrix,ObstacleInforMatrix,flag,predictobstacle,predictCovarianceMatrixSet)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%Co:proximity to obstacles;Cd:deviation from the center path;Cs:smoothness
%criterion
global l_width
global v_width
global vehiclelength
global lamda
CollisionDistThreshold=v_width; %ideal length is:sqrt(v_width^2+vehiclelength^2);                     %unit:m, it is sum of ego vehicle radius and obstacle radius
distanceRefCan=0;maxdistanceRefCan=0;
k_sum=0;maxKCurve=0;
kD_sum=0;maxKDevia=0;
v_sum=0;v_max=max(VelocityProfile);a_sum=0;
distanceObsCan=0;
effectivepoint=0;

[x_temp,y_temp,k_temp,k_curve_deviation]=trajectory_curve_sparse_point(ControlParameter,InitialState);%sparse the path;
CandidatePath=[x_temp',y_temp'];
sampleN=size(x_temp,2);
referencepointnum=size(ReferenceMatrix,1);
k_curve_deviation_max=max(abs(k_curve_deviation));
k_temp_max=max(abs(k_temp));

if size(ObstacleInforMatrix,1)==0
    distanceObsCan=0;
    effectivepoint=sampleN;
else
    for i=1:sampleN
        %%%%%%%%%%%%%%%Calculate Co%%%%%%%%%%%%%%%%%%%
        if flag==1
            [distanceObsCan,CollisionFlag]=DynamicObstacleCostCalculate(ObstacleInforMatrix,CandidatePath(i,:),CollisionDistThreshold,predictobstacle,predictCovarianceMatrixSet,distanceObsCan,lamda);
            if CollisionFlag==1
                break;
            else
                effectivepoint=effectivepoint+1;
            end
        end
        if flag==0
            [distanceObsCan,CollisionFlag]=ObstacleCostCalculate(ObstacleInforMatrix,CandidatePath(i,:),CollisionDistThreshold,distanceObsCan,lamda);
            if CollisionFlag==1
                break;
            else
                effectivepoint=effectivepoint+1;
            end
        end
    end
    distanceObsCan=distanceObsCan/size(ObstacleInforMatrix,1);
end

for i=1:effectivepoint
    %%%%%%%%%%%%%%%Calculate Cd%%%%%%%%%%%%%%%%%%%
    distancetemp2Matrix=bsxfun(@minus,ReferenceMatrix(:,1:2),CandidatePath(i,:));
    distancetemp2Column=sqrt((distancetemp2Matrix(:,1)).^2+(distancetemp2Matrix(:,2)).^2);
    distancetemp2=norm(distancetemp2Column,1)/referencepointnum;
    distanceRefCan=distanceRefCan+distancetemp2;
    
    %%%%%%%%%%%%%%%Calculate Cs%%%%%%%%%%%%%%%%%%%
    if(k_curve_deviation_max~=0 && k_temp_max~=0)
        kD_sum=kD_sum+(k_curve_deviation(i))^2/(k_curve_deviation_max)^2;
        k_sum=k_sum+(k_temp(i))^2/(k_temp_max)^2;
    else
        kD_sum=0;
        k_sum=0;
    end
    
    %%%%%%%%%%%%%%%Calculate Cv%%%%%%%%%%%%%%%%%%%
    v_sum=v_sum+exp(-(VelocityProfile(i).^2/v_max^2));
    a_sum=0;
end

if effectivepoint <= 1%10 %effectivepoint < leastcontrolpoint!!
    Co=1;
    Cd=0;
    Cs=0;
    Cv=0;
else if effectivepoint<=18
        Co=1;
    else
        Co=distanceObsCan/effectivepoint;
    end
    Cd=distanceRefCan/(effectivepoint*l_width*2);
    if Cd>1
        Cd=1;
    end
    Cs=(k_sum+kD_sum)/(effectivepoint*2);%TO-DO锟斤拷set the maximum curvature
    if Cs>1
        Cs=1;
    end
    Cv=(v_sum+a_sum)/(effectivepoint*2);
    if Cv>1
        Cv=1;
    end
end

Cost=[Co,Cd,Cs,Cv];
%W=[0.47,1,0.015];%[wo,wd,ws];
%W=[0.8,0.8,0.015];%[wo,wd,ws];
%W=[1,0.5,0.015];
W=[0.7,0.1,0.1,0.1];%Exp1,2,4
%W=[0.6,0.1,0.1,0.2];

%%%%%%%Result Parameter Table%%%%%%%
%staightline-with-obstacles:[0.47,1,0.015]
%curveline-without-obstacles:[0,1,0];
%curveline-with-obstacles:[0.47,1,0.015]
%%%%%%%%%%%%%%%%%%%%%%%%%%%
TotalCost=Cost*W';%;
%disp(Cost);
end

%% This function calculates the cost of obstacles without prediction
function [distanceObsCan,CollisionFlag]=ObstacleCostCalculate(ObstacleInforMatrix,CandidatePath,CollisionDistThreshold,distanceObsCan,lamda)

%OBSTACLECOSTCALCULATE Summary of this function goes here
%   Detailed explanation goes here
CollisionFlag=0;
for j=1:size(ObstacleInforMatrix,1)
    %% This software only use one circle instead of three circles in the paper.The disadvantage is that the error maybe be enlarged!However this is only used in the simulation!
    distancetemp1=norm(CandidatePath-ObstacleInforMatrix(j,1:2));
    if(distancetemp1< CollisionDistThreshold)
        CollisionFlag=1;
        continue;
    else
        temp=exp(-distancetemp1/lamda);%以10米(x)惩罚为0.1，之后惩罚平滑趋于0的参数是4.3429，lamda=x/log(10);
        distanceObsCan=distanceObsCan+temp;
    end
end

end

%%  This function calculates the cost of obstacles with prediction
function [distanceObsCan,CollisionFlag]=DynamicObstacleCostCalculate(ObstacleInforMatrix,CandidatePathPoint,CollisionDistThreshold,predictobstacle,predictCovarianceMatrixSet,distanceObsCan,lamda)

%OBSTACLECOSTCALCULATE Summary of this function goes here
%   Detailed explanation goes here
CollisionFlag=0;

for j=1:size(ObstacleInforMatrix,1)
    [V, D] = eig(predictCovarianceMatrixSet(:,:,j)); %PayAttention:P是逆矩阵，所以aa是1/D(1),bb是1/D(4)
    CollisionDistThreshold=1/D(1)+CollisionDistThreshold;
    ObstacleInforMatrix(j,:)=predictobstacle(j,:);
    distancetemp1=norm(CandidatePathPoint-ObstacleInforMatrix(j,1:2));
    if(distancetemp1< CollisionDistThreshold)
        CollisionFlag=1;
        continue;
    else
        temp=exp(-distancetemp1/lamda);%以10米(x)惩罚为0.1，之后惩罚平滑趋于0的参数是4.3429，lamda=x/log(10);
        distanceObsCan=distanceObsCan+temp;
    end
end

end
