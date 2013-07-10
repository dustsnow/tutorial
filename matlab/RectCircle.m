rect(2,2,[0,0],0,'r');
circ([2.1,0],1);
print("RectCircle_miss.jpg");
hold off;

rect(2,2,[0,0],0,'r');
circ([2.0,0],1);
print("RectCircle_touch.jpg");
hold off;

rect(2,2,[0,0],0,'r');
circ([1.9,0],1);
print("RectCircle_collide.jpg");
hold off;

