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
% ��������Χ�ж�����һ����Χ�����ӳͷ����ڶ�����Χ�ڽ�������������Χ�ڻ��ǳͷ����趨��Χ��·����������أ�·����Խ�࣬����Խ��
range1 = step*2;
if x-range1<=1,bx1=1;else,bx1 = x-range1;end
if y-range1<=1,by1=1;else,by1 = y-range1;end
if x+range1>=size(map,1),bx2=size(map,1);else,bx2 = x+range1;end
if y+range1>=size(map,2),by2=size(map,2);else,by2 = y+range1;end
B = map(bx1:bx2,by1:by2);
flagdm = find(B==2);
flag = find(B==0);
% �ڶ��μ�ⷶΧ��С ���ӳͷ���������ײ
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
if map(x,y)==0||map(x,y)==2% �ϰ��������ӳͷ�
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*10+field(x,y);
elseif (~isempty(flag))&&isempty(flag1)&&isempty(flagdm)% �ϰ���ڶ�����չ��Χ ������������ 
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))+field(x,y);
elseif ~isempty(flag1)% �ϰ����һ����չ��Χ ���ӳͷ�
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*5+field(x,y);
elseif ~isempty(flagdm1)
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*8+field(x,y);
else % �ϰ����� ���ӳͷ�
    yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2))*1.1+field(x,y);
end
% yout = sqrt(w1*((x-g1)^2 +(y-g2)^2)+w2*((x-s1)^2+(y-s2)^2));

end