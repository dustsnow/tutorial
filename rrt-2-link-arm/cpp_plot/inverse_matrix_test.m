theta1 = 0.1;
theta2 = 0.2;
l1 = 1.0;
l2 = 2.0;
L = [l1;l2];
J = [cos(theta1), cos(theta1+theta2); sin(theta1), sin(theta1+theta2)];
X = J*L

theta = inv(J)*A

