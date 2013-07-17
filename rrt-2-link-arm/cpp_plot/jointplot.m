l1 =1.0; l2 = 2.0;
data = dlmread("path"," ");
[row col] = size(data);

for i=1:1:row
	%subplot(1,2,1)
	plot([data(i,1),data(i,3)],[data(i,2),data(i,4)]);	
	hold on;
	plot(data(i,1),data(i,2),'ro');

%	xlim([-1,1]);
%	ylim([0,2]);

%	subplot(1,2,2);
%    point1_x = l1*cos(data(i,1));
%    point1_y = l1*sin(data(i,1));
%    point2_x = point1_x + l2*cos(data(i,1) + data(i,2));
%    point2_y = point1_y + l2*sin(data(i,1) + data(i,2));
%    point0_x = 0;
%    point0_y = 0;
%    first_arm_x = [point0_x,point1_x];
%    first_arm_y = [point0_y,point1_y];
%    plot(first_arm_x,first_arm_y,'-r');
%    hold on;
%    second_arm_x = [point1_x,point2_x];
%    second_arm_y = [point1_y,point2_y];
%    plot(second_arm_x,second_arm_y,'-b');
%	xlim([-3,3]);
%	ylim([-3,3]);
%	hold off;
%
%	pause(0.001);
end
%subplot(1,2,1);
hold off;
