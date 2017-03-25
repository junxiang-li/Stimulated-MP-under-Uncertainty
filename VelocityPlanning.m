% All the unit of Velocity is m/s
function [vprofile]=VelocityPlanning(XInitial,pathcurvelength)
global number
%% Parameter Initiation
amlat=1;%Acc_{mlat}
dmlon=1;%Dec_{mlon}
ds=0.1;%D_{safe}
kappa=XInitial(:,4);
if(kappa==0)
    kappa=0.001;
end
Sf=abs(pathcurvelength);
%% Calculate the maximal speed
vma=10;%V_{allowed}
vmlat=sqrt(amlat/abs(kappa));%V_{mlat}
vmlon=sqrt(2*(Sf-ds)*dmlon);%V_{mlon}
vlim=min([vma,vmlat,vmlon]);
%% Calculate the desire speed
if Sf-ds <=0
    vdes=0;
else
    vdes=2*vlim/(1+exp(ds-Sf))-vlim;
end
%% Calculate the speed velocity
vprofile(1,1:number)=vdes;
end