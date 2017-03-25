% -------------------------------------------------------------------------
%
% File : DynamicControl.m
%
% Discription : Presume the vehicle fit into Linear Speed Control
%
% Environment : Matlab
%
% Author : John Lee
%
% Copyright (c): 2016 John Lee
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
function [NewDynamicObstacleState]= DynamicControl(ObstacleState)
global iter_t
ObsNum=size(ObstacleState,1);
NewDynamicObstacleState=ObstacleState;
if ObsNum<1
    %disp('Num of Obstacles less than one!');
else
for i=1:ObsNum
    NewDynamicObstacleState(i,1:2)=ObstacleState(i,1:2)+ObstacleState(i,4)*[cos(ObstacleState(i,3)),sin(ObstacleState(i,3))]*iter_t;
    NewDynamicObstacleState(i,3)=ObstacleState(i,3);
end
end
end