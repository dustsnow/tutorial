l1 = 1.0;
l2 = 2.0;
step_size = 0.01;

init_x = 0.1;
init_y = 0.2;
init = [init_x,init_y];

goal_x = -0.5;
goal_y = 2;
goal = [goal_x,goal_y];

last_x = 0.1;
last_y = 0.2;
last = [last_x,last_y];
A = [];
for i=1:2000
	dist = twopointsdist(last,goal);
	new_pt = calcnewpoint(last,goal,step_size);
	last = new_pt;
	A = [A; new_pt];
end
save A.mat A;

[row col] = size(A);
for i=1:1:row-1
	plot([A(i,1),A(i+1,1)],[A(i,2),A(i+1,2)]);	
	hold on;
	%plot(0,0,'o');
	%plot(0.1,0.2,'o');
	%plot(-0.5,2,'o');
	%xlim([-1,1]);
	%ylim([0,2]);
	%hold off;
end
hold off;
