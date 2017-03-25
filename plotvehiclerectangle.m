% -------------------------------------------------------------------------
%
% File : plotvehiclerectangle.m
%
% Discription : This function plot the vehicle.
% Input:v_width represent vehicle_width and vehiclelength represent
% vehicle_length.
% Output:the rectangle has the vehicle_width*vehicle_length
%
% Environment : Matlab
%
% Author : John Lee
%
% Copyright (c): 2016 John Lee
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
function []=plotvehiclerectangle(point,rotate_theta,v_width,vehiclelength,c)
halfvw=v_width/2;
halfvl=vehiclelength/2;
endpoints_rightfront=point+[halfvl,-halfvw]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
endpoints_rightrear=point+[-halfvl,-halfvw]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
endpoints_leftfront=point+[halfvl,halfvw]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
endpoints_leftrear=point+[-halfvl,halfvw]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
rect=[endpoints_leftrear;endpoints_leftfront;endpoints_rightfront;endpoints_rightrear;endpoints_leftrear];
%plot(rect(:,1), rect(:,2),'LineWidth',2,'MarkerFaceColor',c);hold on;
fill(rect(:,1), rect(:,2),c);%,
%set(h,'color',[96 96 96]/255);
%grid on;
end