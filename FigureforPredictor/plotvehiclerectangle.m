function []=plotvehiclerectangle(point,rotate_theta,v_width,vehiclelength,c)
vehiclewidth=v_width;
endpoints_rightfront=point+[vehiclelength,-vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
endpoints_rightrear=point+[-vehiclelength,-vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
endpoints_leftfront=point+[vehiclelength,vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
endpoints_leftrear=point+[-vehiclelength,vehiclewidth]*[cos(rotate_theta),sin(rotate_theta);-sin(rotate_theta),cos(rotate_theta)];
rect=[endpoints_leftrear;endpoints_leftfront;endpoints_rightfront;endpoints_rightrear;endpoints_leftrear];
%plot(rect(:,1), rect(:,2),'k-');hold on;
fill(rect(:,1), rect(:,2),c);
grid on;
end