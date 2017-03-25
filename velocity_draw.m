function velocity_draw(velocitycurve,figVel)
figure(figVel);
x=velocitycurve(1,:);
y=velocitycurve(2,:);
%scatter(x,y);hold on;
values = spcrv([[x(1) x x(end)];[y(1) y y(end)]],3);
plot(values(1,:),values(2,:),'-r','Linewidth',1.5);hold on;
%plot(values(1,:),values(2,:),'--rs','Linewidth',1.5,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10);hold on;
end