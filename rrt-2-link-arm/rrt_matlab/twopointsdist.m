function [dist] = twopointsdist(pt1, pt2)
	x1 = pt1(1);	
	y1 = pt1(2);	
	x2 = pt2(1);	
	y2 = pt2(2);	
	dist = sqrt((x1-x2)**2+(y1-y2)**2);
end
