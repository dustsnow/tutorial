l1 = 1.0;
l2 = 2.0;

goal = [-0.5,2.0];
angle = csvimport('path.csv', 'delimiter',' ','noHeader',true);
for i=67:-1:1
    point1_x = l1*cos(angle{i,1});
    point1_y = l1*sin(angle{i,1});
    point2_x = point1_x + l2*cos(angle{i,1} + angle{i,2});
    point2_y = point1_y + l2*sin(angle{i,1} + angle{i,2});
    point0_x = 0;
    point0_y = 0;
    
    first_arm_x = [point0_x,point1_x];
    first_arm_y = [point0_y,point1_y];
    plot(first_arm_x,first_arm_y,'-r');
    
    hold on;
    
    second_arm_x = [point1_x,point2_x];
    second_arm_y = [point1_y,point2_y];
    plot(second_arm_x,second_arm_y,'-b');
    
    plot(goal(1),goal(2),'o');

    xlim([-3,3]);
    ylim([-3,3]);
    hold off;
    pause(0.1);
%     F(i) = getframe;
end
% movie(F);
    
