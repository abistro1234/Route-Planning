function[flag] = existDynamic(map,start)
x = start(1);
y = start(2);
range1 = 50;
if x-range1<=1,bx1=1;else,bx1 = x-range1;end
if y-range1<=1,by1=1;else,by1 = y-range1;end
if x+range1>=size(map,1),bx2=size(map,1);else,bx2 = x+range1;end
if y+range1>=size(map,2),by2=size(map,2);else,by2 = y+range1;end
B = map(bx1:bx2,by1:by2);
flag = find(B==2);
end