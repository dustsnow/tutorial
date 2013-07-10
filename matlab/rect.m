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
	leftbot_corner_x   = center(1) - x/2;
	leftbot_corner_y   = center(2) - y/2;

	lefttop_corner_x   = leftbot_corner_x;
	lefttop_corner_y   = leftbot_corner_y + y;
	
	rightbot_corner_x  = leftbot_corner_x + x;
	rightbot_corner_y  = leftbot_corner_y;

	rightop_corner_x   = leftbot_corner_x + x;
	rightop_corner_y   = leftbot_corner_y + y;
	
	% Roate the rectangle
	rect_x = [leftbot_corner_x,lefttop_corner_x,rightop_corner_x,rightbot_corner_x,leftbot_corner_x];
	rect_y = [leftbot_corner_y,lefttop_corner_y,rightop_corner_y,rightbot_corner_y,leftbot_corner_y];

	rect = rotation_matrix * [rect_x;rect_y];

	plot(rect(1,:),rect(2,:),color);
	hold on;
	xlim([-2,5]);
	ylim([-2,5]);
	%axis equal;
end

