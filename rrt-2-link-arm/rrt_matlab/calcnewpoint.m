function [new_pt] = calcnewpoint(pt1,pt2,step_size)
	x1 = pt1(1);	
	y1 = pt1(2);	
	x2 = pt2(1);	
	y2 = pt2(2);	
	dist = twopointsdist(pt1,pt2);
	new_pt_x = step_size/dist*(x2-x1) + x1;
	new_pt_y = step_size/dist*(y2-y1) + y1;
	new_pt = [new_pt_x,new_pt_y];
end
