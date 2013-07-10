rect(2,2,[0,0],0,'r');
rect(2,2,[2.1,0],0,'b');
print("RectRect_miss.jpg");
hold off;

rect(2,2,[0,0],0,'r');
rect(2,2,[2.0,0],0,'b');
print("RectRect_touch.jpg");
hold off;

rect(2,2,[0,0],0,'r');
rect(2,2,[1.9,0],0,'b');
print("RectRect_collide.jpg");
hold off;

rect(2,2,[0,0],0,'r');
rect(2,2,[sqrt(8)/2+1.1,0],45,'b');

hold off;
