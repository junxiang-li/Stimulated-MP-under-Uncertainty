clc;clear;
figure();
for i=1:2
    if i==1
        o1=load('./velocityprofile/Exp4');
        velocitycurve1=o1.velocitycurve;
    else
        o2=load('./velocityprofile/Exp4p');
        velocitycurve1=o2.velocitycurve;
    end
    x=velocitycurve1(1,:);
    y=velocitycurve1(2,:);
    values=[x;y];
    values = spcrv([[x(1) x x(end)];[y(1) y y(end)]],3);
    m=ones(2,size(x,2));
    for j=1:size(x,2)
        m(1,j)=j;
        m(2,j)=values(2,2+8*(j-1));%4:Exp1_2;8:else
    end
    if i==1
        plot(m(1,:),m(2,:),'.-r','Linewidth',1.75);hold on;
        %plot(values(1,:),values(2,:),'-r','Linewidth',1.5);hold on;
    else
        plot(m(1,:),m(2,:),'.-b','Linewidth',1.75);hold on;
        %plot(values(1,:),values(2,:),'-b','Linewidth',1.5);hold on;
    end
end
set(gca, 'FontSize', 18); %<- Set properties
xlabel('Planning Cycle(#)');
ylabel('Velocity(m/s)');
axis([1,size(m,2),1,3.5]);%0.5
legend('ILTP','KPMP');
print('Simulation','-dpng','-r300');









%plot(values(1,:),values(2,:),'--rs','Linewidth',1.5,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10);hold on;