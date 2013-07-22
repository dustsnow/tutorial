l1 = 1;
l2 = 2;
theta1 = 0.1;
theta2 = 0.2;

xGoal = [-0.5, 2.0];
xi = [l1*cos(theta1) + l2*cos(theta1+theta2), l1*sin(theta1) + l2*sin(theta1+theta2)];
dx = xGoal - xi;
