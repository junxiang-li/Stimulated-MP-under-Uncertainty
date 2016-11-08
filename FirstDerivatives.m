function [dhdq,state_terminal]= FirstDerivatives(q0,state_initial)
%The input should be the row vector.

%q0=[   -1.0181    0.0318   -0.0001 -197.1080];
%state_initial=[   40.0000   -0.3571         0         0];
b=q0(1);
c=q0(2);
d=q0(3);
sf=q0(4);

kf1=b+2*c*sf+3*d*(sf^2);
kf=b*sf+c*(sf^2)+d*(sf^3);
thetaf=b*sf.^2/2+c*sf.^3/3+d*sf.^4/4;
ds=0.001;
s=0:ds:sf;
C(1)=trapz(cos(b*s.^2/2+c*s.^3/3+d*s.^4/4))*ds;%x(s)
S(1)=trapz(sin(b*(s.^2)/2+c*(s.^3)/3+d*(s.^4)/4))*ds;%y(s)  

for i=2:4
    S(i)=-1/i*trapz(s.^i.*sin(b*s.^2/2+c*s.^3/3+d*s.^4/4))*ds;
    C(i)=1/i*trapz(s.^i.*cos(b*s.^2/2+c*s.^3/3+d*s.^4/4))*ds;
end

%Change when it is necessary%
dhdq=[S(2) S(3) S(4) cos(thetaf); C(2) C(3) C(4) sin(thetaf);sf^2/2 sf^3/3 sf^4/4 kf;sf sf^2 sf^3 kf1;];
i_h=[C(1);S(1);thetaf;kf;];
state_terminal=i_h'+state_initial;
end