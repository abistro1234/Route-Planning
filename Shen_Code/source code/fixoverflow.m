%% 该函数用于演示BAS应用于路径规划
    %当天牛跑到界外时，相当于跑一个非常大的反馈

%% 清空环境
clc
clear
close all%关闭所有图像

%% 数据初始化

x=18;%走多远
eta=0.99993;
step=4;%步长
n=50000;%迭代次数
%p=0.3; %障碍物碰撞的惩罚项设置

ymax=x; %y的上限,和x相同是为了美观
ymin=0;  %y的下限
Z=peaks(x); %形成障碍物
t=10; %最后的图像演示次数
f=5;  %图像的帧率

bestfitness=inf; %适应度无限大
c=5; %步长和须的比例（每次改变步长就能改变须的长度了）


y=rands(x,1); 

%% 生成路径
path = ymin + (ymax-ymin)*rand(1,x); %生成随机路径
path = roundn(path,-2);%取小数点后两位
path(1)=x/2; path(x)=x/2;%固定起点和终点
fitness = cacufit(Z,path,x);%计算适应度
BestFitness=bestfitness;%保存的路径更新（非实时路径）


if fitness < bestfitness
    bestpath=path;
    bestfitness=cacufit(Z,bestpath,x);
end
BestPath = bestpath;
display(['0:','bestpath=[',num2str(bestpath),'],bestfitness=',num2str(bestfitness)])

d0 = step/c;
tic
for ii=1:n %
%% 天牛须建模1  
    %探索后进行移动，但是陷入死循环，同时进行21维的移动（y、z方向），很难满足约束条件
    dir=rands(x,1);   %须的方向。因为横切有21层，所以我们设置为21维. 
    dir=dir'/(eps+norm(dir'));       %归一化
    dir=roundn(dir,-2); dir(1)=0; dir(x)=0;
    leftpath=path+dir*d0/2;
    leftfitness=cacufit(Z,leftpath,x);
    rightpath=path-dir*d0/2;
    rightfitness=cacufit(Z,rightpath,x); 
    path=path-step*dir*sign(leftfitness-rightfitness);
    %计算适应度
    fitness=cacufit(Z,path,x);
    if fitness<bestfitness
        bestfitness=fitness;
        bestpath=path;
    end
%BestFitness=[BestFitness;bestfitness];
%BestPath=[BestPath;bestpath];
    step=step*eta;
end
toc

%% 适应度变化


figure(1)
hold on
plot(bestpath)
%k=size(BestPath,1);
%plot(BestPath(k,:))
axis([0 x 0 x]);
grid on
title('路径变化示意')
xlabel('横向距离')
ylabel('纵向距离')
axis equal
obstacle(x)
hold off

bestfitness

save('matPath.mat','BestPath','bestfitness')

%nbmovie(x,t,f,BestPath)



%% 实验记录
% 跑出了迄今为止最好的可以绕过障碍物的结果

