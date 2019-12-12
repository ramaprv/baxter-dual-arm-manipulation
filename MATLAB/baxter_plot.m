
clc;clear all;close all;
p0 = [10,10,10];
a1 = [10,10,280.35];
p1 = [10,79,280.35];

p2 = [10,159,280.35];
a2 = [10,435.35,280.35];
p3 = [10,435.35,211.35];
p4 = [10,515.35,211.35];
a3 = [10,809.64,211.35];
p5 = [10,809.64,201.35];
p6 = [10,1039.165,201.35];
p7 = [10,1119.165,201.35];

pts=[p0;a1];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p0(1),p0(2),p0(3),'  S0','VerticalAlignment','bottom','HorizontalAlignment', 'left')
hold on

pts=[a1;p1];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p1(1),p1(2),p1(3),'S1','VerticalAlignment','top')
hold on

pts=[p1;p2];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p2(1),p2(2),p2(3),'   E0','VerticalAlignment','bottom')
hold on

pts=[p2;a2];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
% text(a2(1),a2(2),a2(3))
hold on

pts=[a2;p3];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p3(1),p3(2),p3(3),'E1','VerticalAlignment','top')
hold on

pts=[p3;p4];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p4(1),p4(2),p4(3),'W0','VerticalAlignment','top')
hold on

pts=[p4;a3];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
% text(a3(1),a3(2),a3(3))
hold on

pts=[a3;p5];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p5(1),p5(2),p5(3),'W1 ','VerticalAlignment','top')
hold on

pts=[p5;p6];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p6(1),p6(2),p6(3),'W2','VerticalAlignment','top')
hold on

pts=[p6;p7];
plot3(pts(:,1),pts(:,2),pts(:,3),'-o','linewidth',2.5)
text(p7(1),p7(2),p7(3),'   End Effector')
hold on

axis([-100 500 0 1500 -200 500]);
grid on
view([90,0])