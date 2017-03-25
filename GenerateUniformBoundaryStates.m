function [ BoundaryStatePairSet ] = GenerateUniformBoundaryStates( FinalPoint,l_width,v_width,n_p,n_l,XInitial )
%GENERATEUNIFORMBOUNDARYSTATES Summary of this function goes here
%   Detailed explanation goes here
BoundaryStatePairSet=zeros(4,n_p*n_l);
%finalcurvekappa=0;%finalcurvekappa=FinalPose(4);
for i=0:(n_p-1)
    for j=0:(n_l-1)
        n=i*n_l+j;
        offset=-0.5*(l_width-v_width)+(l_width-v_width)*i/(n_p-1);
        finalangle=FinalPoint(3,:);
        finalx=FinalPoint(1,:)-offset*sin(finalangle);
        finaly=FinalPoint(2,:)+offset*cos(finalangle);
        finalcurvekappa=FinalPoint(4,:);
        BoundaryStatePairSet(:,n+1)=[finalx;finaly;finalangle;finalcurvekappa];
    end
end
%plot(XInitial(:,1),XInitial(:,2),'o');
% I=imread('vehicle.png');
% b=imrotate(I,-90-180*XInitial(3)/3.14,'nearest');
% imshow(b, 'XData',[XInitial(1)-1,XInitial(1)+1], 'YData', [XInitial(2)-0.5,XInitial(2)+0.5]);hold on;
% plot(BoundaryStatePairSet(1,:),BoundaryStatePairSet(2,:),'g*');
end

