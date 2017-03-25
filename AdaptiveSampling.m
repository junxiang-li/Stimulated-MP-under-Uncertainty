% -------------------------------------------------------------------------
%
% File : AdaptiveSampling.m
%
% Discription : Examples for adaptive sampling
%
% Environment : Matlab
%
% Author : John Lee
%
% Copyright (c): 2016 John Lee
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

clc;clear;
global number
global iter_t
global l_width
global v_width
global vehiclelength

load full_tree_onelayer.mat
figSim=figure('NumberTitle','off','Name','AdaptiveSampling');
figure(figSim);

d_horizon=50; %horizon index(50 points ahead, not the length)
endflag=0;%0 means normal,1 means stop!
%%%%%%%Function GenerateUniformBoundaryStates() Parameter%%%%%%%%%%%%%%%%
l_heading=0;%the parameter could be tested by the perception system
l_width=4;% the width of the lane
v_width=1; % the width of the vehicle
vehiclelength=2;% the length of the vehicle
d=10;% horizon forward
n_p=15;% the number of lateral offsets to generate for each lane
n_l=1;% the number of lanes itself
%%%%%%Function Parametric_Trajectory_Generation() Parameter%%%%%%%%%%%%%%%%
parameter_previous=[0,0,0,0]';
LookUpTable=lookup_table;
InitialPosDB=[40,0,0,0;
    46.5,3.6,0.8,0;
    55,21,0.8,0;
    68,24,0,0;
    ];
for i=1:4
    %%%%%%%%%%%%%%%%% Display the Environment %%%%%%%%%%%%%%%%
    subplot(2,2,i);
    FinalPose=InitialPosDB(i,:);
    referencepath_xy=plotRoad3(figSim);
    plotvehiclerectangle([FinalPose(1),FinalPose(2)],FinalPose(3),v_width,vehiclelength,'r');
    %% Initialization-Sample
    MinCost=1;
    parameter_inherit_flag=0;
    XInitial=FinalPose;%[x,y,\theta,\kappa]
    XInitialPosition=[XInitial(1),XInitial(2)]';
    [FinalPoint,endflag]=JudgeFinalCenterPoint(referencepath_xy,XInitial',d_horizon);
    XF=GenerateUniformBoundaryStates(FinalPoint,l_width,v_width,n_p,n_l,XInitial);
    referencepath=TrimReferencePath(referencepath_xy,XInitialPosition,FinalPoint);
    SamplePointNum=size(XF,2);
    U=zeros(4,SamplePointNum);
    %% Generation
    for i=1:SamplePointNum
        parameter_previous=ParameterInitiation(XInitial,XF(:,i)',LookUpTable);
        [parameter,final_state]=Parametric_Trajectory_Generation(parameter_previous',XInitial,XF(:,i)',parameter_inherit_flag);
        trajectory_curve_draw(parameter,XInitial);
    end
    axis([FinalPose(1)-5,FinalPose(1)+15,FinalPose(2)-6,FinalPose(2)+8]);
    xlabel('x');ylabel('y');
    grid off;
end

