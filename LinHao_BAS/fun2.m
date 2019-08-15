function yout=fun2(xbest,goal,nodes,map,field,step)
% mapname = 'map1.bmp';
% map=im2bw(imread(mapname)); % input map read from a bmp file. for new maps write the file name here
% map = map';
w1=0.1;
w2=1;
theta = xbest;
x = theta(1);
y = theta(2);
s = nodes(end,:);
s1 = s(1);
s2 = s(2);
g1 = goal(1);
g2 = goal(2);
% 加两个范围判定，第一个范围内增加惩罚，第二个范围内奖励，第三个范围内还是惩罚。设定范围和路径点数量相关，路径点越多，长度越短
range1 = step*2;
if x-range1<=1,bx1=1;else,bx1 = x-range1;end
if y-range1<=1,by1=1;else,by1 = y-range1;end
if x+range1>=size(map,1),bx2=size(map,1);else,bx2 = x+range1;end
if y+range1>=size(map,2),by2=size(map,2);else,by2 = y+range1;end
B = map(bx1:bx2,by1:by2);
flagdm = find(B==2);
flag = find(B==0);
% 第二次检测范围较小 增加惩罚，避免碰撞
range2 = step;
if x-range2<=1,bx1=1;else,bx1 = x-range2;end
if y-range2<=1,by1=1;else,by1 = y-range2;end
if x+range2>=size(map,1),bx2=size(map,1);else,bx2 = x+range2;end
if y+range2>=size(map,2),by2=size(map,2);else,by2 = y+range2;end
B1 = map(bx1:bx2,by1:by2);
flagdm1 = find(B1==2);
flag1 = find(B1==0);
% yout=-sin(x).*(sin(x.^2/pi)).^20-sin(y).*(sin(2*y.^2/pi)).^20;
% yout = sqrt((x(:,1)-y(:,1)).^2 + (x(:,2)-y(:,2)).^2 );
if map(x,y)==0||map(x,y)==2% 障碍物内增加惩罚
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*10+field(x,y);
elseif (~isempty(flag))&&isempty(flag1)&&isempty(flagdm)% 障碍物第二个扩展范围 鼓励行走区域 
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))+field(x,y);
elseif ~isempty(flag1)% 障碍物第一个扩展范围 增加惩罚
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*5+field(x,y);
elseif ~isempty(flagdm1)
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*8+field(x,y);
else % 障碍物外 增加惩罚
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*1.1+field(x,y);
end
% yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2));

end