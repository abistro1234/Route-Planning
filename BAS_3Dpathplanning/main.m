%%%%%%%%%%%%%%%%%%%%%%% 
% 本main文档包含3个主程序的拼接，按顺序排列为ACO、BAS及ACO与BAS结合算法
% 迭代次数均为500（其中ACO算法200步几乎达到最优解）
% BAS算法对初始路径较为敏感
%%%%%%%%%%%%%%%%%%%%%%%
%% ACO算法
clc
clear all
close all

%% 数据初始化

%下载数据
load  HeightData
tic
%网格划分
LevelGrid=10;
PortGrid=21;

%起点终点网格点 
starty=10;starth=4;
endy=8;endh=5;
m=1;
%算法参数
PopNumber=20;         %种群个数
BestFitness=[];    %最佳个体

%初始信息素
pheromone=ones(21,21,21);

%% 初始搜索路径
[path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,pheromone, ...
    HeightData,starty,starth,endy,endh); 
fitness=CacuFit(path);                          %适应度计算
[bestfitness,bestindex]=min(fitness);           %最佳适应度
bestpath=path(bestindex,:);                     %最佳路径
BestFitness=[BestFitness;bestfitness];          %适应度值记录
 
%% 信息素更新
rou=0.2;
cfit=100/bestfitness;
for i=2:PortGrid-1
    pheromone(i,bestpath(i*2-1),bestpath(i*2))= ...
        (1-rou)*pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
end
    
%% 循环寻找最优路径
for kk=1:500
     
    %% 路径搜索
    [path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,...
        pheromone,HeightData,starty,starth,endy,endh); 
    
    %% 适应度值计算更新
    fitness=CacuFit(path);                               
    [newbestfitness,newbestindex]=min(fitness);     
    if newbestfitness<bestfitness
        bestfitness=newbestfitness;
        bestpath=path(newbestindex,:);
    end 
    BestFitness=[BestFitness;bestfitness];
    
    %% 更新信息素
    cfit=100/bestfitness;
    for i=2:PortGrid-1
        pheromone(i,bestpath(i*2-1),bestpath(i*2))=(1-rou)* ...
            pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
    end
%   display([num2str(kk),'bestpath=[',num2str(bestpath),'],bestfitness=',num2str(bestfitness)])
end
bestfitness
toc
disp(['运行时间: ',num2str(toc)]); 
%% 最佳路径
for i=1:21
    a(i,1)=bestpath(i*2-1);
    a(i,2)=bestpath(i*2);
end
%% 俯视图
figure(1)
x=1:21;
y=1:21;
[x1,y1]=meshgrid(x,y);
mesh(x1,y1,HeightData)
colormap(jet);
axis([1,21,1,21,0,2000])
hold on
k=1:21;
plot3(k(1)',a(1,1)',a(1,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10)
plot3(k(21)',a(21,1)',a(21,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','y',...
                       'MarkerSize',10)
                   text(k(1)',a(1,1)',a(1,2)'*200,'S');
text(k(21)',a(21,1)',a(21,2)'*200,'T');
xlabel('km','fontsize',12);
ylabel('km','fontsize',12);
zlabel('m','fontsize',12);
title('Three-dimensional path planning space vertical view','fontsize',12)
set(gcf, 'Renderer', 'ZBuffer')
hold on
plot3(k',a(:,1)',a(:,2)'*200,'--bo','Color',[0 0 1]);
view(0,90)%'俯视图'
hold on
%% 3D视图
figure(2)
x=1:21;
y=1:21;
[x1,y1]=meshgrid(x,y);
mesh(x1,y1,HeightData)
colormap(jet);
axis([1,21,1,21,0,2000])
hold on
k=1:21;
plot3(k(1)',a(1,1)',a(1,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10)
plot3(k(21)',a(21,1)',a(21,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','y',...
                       'MarkerSize',10)
                   text(k(1)',a(1,1)',a(1,2)'*200,'S');
text(k(21)',a(21,1)',a(21,2)'*200,'T');
xlabel('km','fontsize',12);
ylabel('km','fontsize',12);
zlabel('m','fontsize',12);
title('Three-dimensional path planning space','fontsize',12)
set(gcf, 'Renderer', 'ZBuffer')
hold on
p1=plot3(k',a(:,1)',a(:,2)'*200,'--bo','Color',[0 0 1]);
hold on
%% 去掉3D空间模型的剖视图
figure(3)
x=1:21;
y=1:21;
[x1,y1]=meshgrid(x,y);
axis([1,21,1,21,0,2000])
grid on
hold on
k=1:21;
plot3(k(1)',a(1,1)',a(1,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10)
plot3(k(21)',a(21,1)',a(21,2)'*200,'--o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','y',...
                       'MarkerSize',10)
                   text(k(1)',a(1,1)',a(1,2)'*200,'S');
text(k(21)',a(21,1)',a(21,2)'*200,'T');
xlabel('km','fontsize',12);
ylabel('km','fontsize',12);
zlabel('m','fontsize',12);
title('Three-dimensional path planning space','fontsize',12)
set(gcf, 'Renderer', 'ZBuffer')
hold on
p2=plot3(k',a(:,1)',a(:,2)'*200,'--bo','Color',[0 0 1]);
view(0,0);
hold on

%% 适应度变化
figure(4)
plot(BestFitness,'LineWidth',2,'Color',[0 0 1]);
title('Optimal fitness value trend')
xlabel('Iteration')
ylabel('Fitness value')
hold on

%% BAS
% clc
% clear all
% close all

%% 数据初始化
tic
%下载数据
load  HeightData
%网格划分
LevelGrid=10;%
PortGrid=21;

%起点终点网格点 
starty=10;starth=4;
endy=8;endh=5;

%算法参数
bestfitness=inf;
BestFitness=[]; 
bestpath=zeros(1,42);
bestfitness_store=[];
path_store=[];

eta = 0.998;
step0=18;
eta2=0.995;
eta0=0.998;

c=1;       
step=6;   
tempstep=0;

n=500;   
k=PortGrid; 
z=1;
pre=0;lat=0;
%% 生成路径并判定是否合法
    if starty<endy
        pre=starty;lat=endy;
    else 
        pre=endy;lat=starty;
    end
while(1)
    path1=round(unifrnd(round(pre),round(lat),1,21)); 
    path2=round(unifrnd(2,10,1,21));  
    path=reshape([path1;path2],1,42);
    if checkpath(path, HeightData)==1
        break;
    end
    z=z+1;
    if pre<=1||lat>=20
        pre=1;lat=20;
    else
%         pre=0.999999*pre;lat=1.000001*lat;
         pre=pre-0.0005;lat=lat+0.0005;
    end
end
% z
    path(1,1:2)=[starty;starth];
    path(1,41:42)=[endy;endh];

    bestpath=path;
    bestfitness=CacuFit(bestpath);
%     display([num2str(1),'bestpath=[',num2str(bestpath),'],bestfitness=',num2str(bestfitness)])

    bestfitness_store=bestfitness;
    path_store=[0;path';bestpath'];
    
%% 天牛须建模1
for ii=1:n
    d0 =step/c;
    stepnew=step;
%    r=0;
    while(1)
        dir=rands(PortGrid*2,1);   
        dir=(dir/norm(dir))';      
        leftpath=path-round(d0*10*dir/2);
            leftfitness=CacuFit(leftpath);
            rightpath=path+round(d0*10*dir/2);
                rightfitness=CacuFit(rightpath); 
                path=path-round(stepnew*dir*sign(rightfitness-leftfitness));
                   path(1,1:2)=[starty;starth];
                   path(1,41:42)=[endy;endh];
                if checkpath(path, HeightData)==1
                     break;
                else
                     path=bestpath;  
                     stepnew=stepnew*eta2;
%                     r=r+1;
                     if stepnew<=1%||r>=800
                         break;
                     end
                end
       end  
    %%%%%%%%%%% smooth path         
    for iii=2:20
        if ((path(2*iii+1)-path(2*iii-1))^2 >=(step0)^2)
            path(2*iii-1)=path(2*(iii-1)-1);
            path(2*iii+1)=path(2*(iii-1)+1);
        end
    end
      step0=eta0*step0;  
    %计算适应度
        fitness=CacuFit(path);
        if fitness<bestfitness
            bestfitness=fitness;
            bestpath=path;
        end
%    path_store=cat(2,path_store,[ii;path;fitness]);
    bestfitness_store=[bestfitness_store;bestfitness];
%     display([num2str(ii),'bestpath=[',num2str(bestpath),'],bestfitness=',num2str(bestfitness)])
    BestFitness=bestfitness_store;
    step=step*eta;  
end
bestfitness
toc
disp(['运行时间: ',num2str(toc)]); 
%% 最佳路径
%TODO:修改适应度变化的画图部分代码
for i=1:21
    a(i,1)=bestpath(i*2-1);
    a(i,2)=bestpath(i*2);
end
figure(1)
k=1:21;
plot3(k',a(:,1)',a(:,2)'*200,'--ro','Color',[1 0 0]);
hold on

figure(2)
k=1:21;
plot3(k',a(:,1)',a(:,2)'*200,'--ro','Color',[1 0 0]);
hold on

figure(3)
k=1:21;
plot3(k',a(:,1)',a(:,2)'*200,'--ro','Color',[1 0 0]);
hold on

%% 适应度变化
figure(4)
plot(BestFitness,'LineWidth',2,'Color',[1 0 0]);
hold on

%% ACO-BAS
tic
%网格划分
LevelGrid=10;%
PortGrid=21;

%起点终点网格点 
starty=10;starth=4;
endy=8;endh=5;

%初始信息素
pheromone=ones(21,21,21);
PopNumber=10; %种群个体

%算法参数
bestfitness=inf;
BestFitness=[]; 

eta = 0.9999;%步长的衰减率
eta0=0.998;
c=2;      

step=4;     
step0=21;
n=500;     %iterations
k=PortGrid;  %维数就是要走的层数
z=1;

%% 初始搜索路径
[path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,pheromone, ...
    HeightData,starty,starth,endy,endh); 
fitness=CacuFit(path);                          %适应度计算
[bestfitness,bestindex]=min(fitness);           %最佳适应度
bestpath=path(bestindex,:);                     %最佳路径
BestFitness=[BestFitness;bestfitness];          %适应度值记录
path1=bestpath';
path=reshape(path1,1,42);
bestpath=path;
bestfitness_store=bestfitness;
% path_store=[0;path;bestpath];



for ii=1:n
%% 天牛须建模1
    d0 =step/c;

    while(1)
        dir=rands(PortGrid*2,1);   %须的方向。因为横切有21层，所以我们设置为21维；y与h坐标加起来是42维
        dir=(dir/norm(dir))';       %归一化    各个方向上的维度取值是[-1,1]，乘十后四舍五入为整数
        leftpath=path+round(d0*10*dir/2);
             path(1,1:2)=[starty;starth];
             path(1,41:42)=[endy;endh];
            leftfitness=CacuFit(leftpath);
            rightpath=path-round(d0*10*dir/2);
                 path(1,1:2)=[starty;starth];
                 path(1,41:42)=[endy;endh];
                rightfitness=CacuFit(rightpath); 
                path=path-round(step*dir*sign(leftfitness-rightfitness));
                   path(1,1:2)=[starty;starth];
                   path(1,41:42)=[endy;endh];
                if checkpath(path, HeightData)==1
                     break;
                else
                     path=bestpath;  
                     step=step*eta;
                     if step<=1
                         break;
                     end
                end
       end 
	%顺滑路径，剔除无效路径点
for iii=2:20
    if ((path(2*iii+1)-path(2*iii-1))^2 >=(step0)^2)
        path(2*iii-1)=path(2*(iii-1)-1);
        path(2*iii+1)=path(2*(iii-1)+1);
    end
end              
        
    %计算适应度
    fitness=CacuFit(path);
    if fitness<bestfitness
        bestfitness=fitness;
        bestpath=path;
    end

%     path_store=cat(2,path_store,[ii;path';fitness]);
     bestfitness_store=[bestfitness_store;bestfitness];
%     display([num2str(ii),'bestpath=[',num2str(bestpath),'],bestfitness=',num2str(bestfitness)])
%     step=step*eta;
BestFitness=bestfitness_store; 
end
bestfitness
% step
% step0
toc
disp(['运行时间: ',num2str(toc)]); 
for i=1:21
    a(i,1)=bestpath(i*2-1);
    a(i,2)=bestpath(i*2);
end
figure(1)
k=1:21;
plot3(k',a(:,1)',a(:,2)'*200,'--ko','Color',[0.5 0.5 0.5]);

figure(2)
k=1:21;
plot3(k',a(:,1)',a(:,2)'*200,'--ko','Color',[0.5 0.5 0.5]);

figure(3)
k=1:21;
plot3(k',a(:,1)',a(:,2)'*200,'--ko','Color',[0.5 0.5 0.5]);

%% 适应度变化
figure(4)
plot(BestFitness,'LineWidth',2,'Color',[0.5 0.5 0.5]);