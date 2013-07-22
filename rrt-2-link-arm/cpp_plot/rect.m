% x is the length
% y is the width
% center:
%	center[x,y];
%	The coordinate of the center point
% theta:
% 	The angle in degree rotate from a vertical-standing rectangle.
% Color:
% 	The matlab color type such as 'r', 'b'

function [] = rect(x,y,center,theta,color)
	theta = theta/180*pi;
	rotation_matrix = [cos(theta),-sin(theta);sin(theta),cos(theta)];
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Calculate the coordinates of a vertical-standing rectangle

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	leftbot_corner_x   = -x/2;
	leftbot_corner_y   = -y/2;

	lefttop_corner_x   = -x/2;
	lefttop_corner_y   = y/2;
	
	rightbot_corner_x  = x/2;
	rightbot_corner_y  = -y/2;

	rightop_corner_x   = x/2;
	rightop_corner_y   = y/2;

	rect_x = [leftbot_corner_x,lefttop_corner_x,rightop_corner_x,rightbot_corner_x,leftbot_corner_x];
	rect_y = [leftbot_corner_y,lefttop_corner_y,rightop_corner_y,rightbot_corner_y,leftbot_corner_y];

	rect = rotation_matrix * [rect_x;rect_y];
	rect(1,:) = rect(1,:)+center(1);
	rect(2,:) = rect(2,:)+center(2);
	
	plot(rect(1,:),rect(2,:),color);
	hold on;
	xlim([-2,5]);
	ylim([-2,5]);
	%axis equal;
end

