function [FinalPoint,endflag] = JudgeFinalCenterPoint( referencepath_xy,XInitial,d_horizon )
%JUDGEFINALCENTERPOINT Summary of this function goes here
%   Detailed explanation goes here
endflag=0;
n=size(referencepath_xy,2);
XInitial=XInitial*ones(1,n);
R=referencepath_xy(1:2,:)-XInitial(1:2,:);
tempresult=sqrt(R(1,:).^2+R(2,:).^2);
Index=find(tempresult==min(tempresult),1);
lookforwardindex = Index + d_horizon;%d_horizon is the forward index
if lookforwardindex<n
    FinalPoint= referencepath_xy(:,lookforwardindex);
else
    FinalPoint= referencepath_xy(:,end);
    endflag=1;
end
end

