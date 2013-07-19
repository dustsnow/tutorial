l1 =1.0; l2 = 2.0;l3 = 1.0;
w = 0.1;
data = dlmread("mypath"," ");
[row col] = size(data);
% the for loop is inversed because the path is found from end to start
for i=row:-1:1
%	subplot(1,2,1)
%	plot([data(i,1),data(i,3)],[data(i,2),data(i,4)]);	
%	hold on;
%	plot(data(i,1),data(i,2),'ro');

	%subplot(1,2,2);
	rect(0.4,0.4,[2.0,2.0],0,'r');
    point0_x = 0;
    point0_y = 0;
    point1_x = l1*cos(data(i,1));
    point1_y = l1*sin(data(i,1));
    point2_x = point1_x + l2*cos(data(i,1) + data(i,2));
    point2_y = point1_y + l2*sin(data(i,1) + data(i,2));
    point3_x = point2_x + l3*cos(data(i,1) + data(i,2) + data(i,3));
    point3_y = point2_y + l3*sin(data(i,1) + data(i,2) + data(i,3));

	cp1_x = l1/2.0*cos(data(i,1));
	cp1_y = l2/2.0*sin(data(i,1));
	cp2_x = l1*cos(data(i,1))+l2/2.0*cos(data(i,1) + data(i,2));
	cp2_y = l1*sin(data(i,1))+l2/2.0*sin(data(i,1) + data(i,2));
	cp3_x = l1*cos(data(i,1))+l2*cos(data(i,1) + data(i,2))+l3/2.0*cos(data(i,1) + data(i,2) + data(i,3));
	cp3_y = l1*sin(data(i,1))+l2*sin(data(i,1) + data(i,2))+l3/2.0*sin(data(i,1) + data(i,2) + data(i,3));

	link1_center_x = point0_x+(point1_x - point0_x)/2;
	link1_center_y = point0_y+(point1_y - point0_y)/2;
	link2_center_x = point1_x+(point2_x - point1_x)/2;
	link2_center_y = point1_y+(point2_y - point1_y)/2;
	link3_center_x = point2_x+(point3_x - point2_x)/2;
	link3_center_y = point2_y+(point3_y - point2_y)/2;

	rect(l1,w,[link1_center_x,link1_center_y],data(i,1)*180/pi,'r');
	rect(l2,w,[link2_center_x,link2_center_y],(data(i,1)+data(i,2))*180/pi,'r');
	rect(l3,w,[link3_center_x,link3_center_y],(data(i,1)+data(i,2)+data(i,3))*180/pi,'r');

    first_arm_x = [point0_x,point1_x];
    first_arm_y = [point0_y,point1_y];
    plot(first_arm_x,first_arm_y,'-r');
    hold on;

    second_arm_x = [point1_x,point2_x];
    second_arm_y = [point1_y,point2_y];
    plot(second_arm_x,second_arm_y,'-b');

    third_arm_x = [point2_x,point3_x];
    third_arm_y = [point2_y,point3_y];
    plot(third_arm_x,third_arm_y,'-r');

	xlim([-6,6]);
	ylim([-6,6]);
	hold off;


	%pause();
	pause(0.001);
end
%subplot(1,2,1);
axis equal;
hold off;
