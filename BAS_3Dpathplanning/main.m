%%%%%%%%%%%%%%%%%%%%%%% 
% ��main�ĵ�����3���������ƴ�ӣ���˳������ΪACO��BAS��ACO��BAS����㷨
% ����������Ϊ500������ACO�㷨200�������ﵽ���Ž⣩
% BAS�㷨�Գ�ʼ·����Ϊ����
%%%%%%%%%%%%%%%%%%%%%%%
%% ACO�㷨
clc
clear all
close all

%% ���ݳ�ʼ��

%��������
load  HeightData
tic
%���񻮷�
LevelGrid=10;
PortGrid=21;

%����յ������ 
starty=10;starth=4;
endy=8;endh=5;
m=1;
%�㷨����
PopNumber=20;         %��Ⱥ����
BestFitness=[];    %��Ѹ���

%��ʼ��Ϣ��
pheromone=ones(21,21,21);

%% ��ʼ����·��
[path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,pheromone, ...
    HeightData,starty,starth,endy,endh); 
fitness=CacuFit(path);                          %��Ӧ�ȼ���
[bestfitness,bestindex]=min(fitness);           %�����Ӧ��
bestpath=path(bestindex,:);                     %���·��
BestFitness=[BestFitness;bestfitness];          %��Ӧ��ֵ��¼
 
%% ��Ϣ�ظ���
rou=0.2;
cfit=100/bestfitness;
for i=2:PortGrid-1
    pheromone(i,bestpath(i*2-1),bestpath(i*2))= ...
        (1-rou)*pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
end
    
%% ѭ��Ѱ������·��
for kk=1:500
     
    %% ·������
    [path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,...
        pheromone,HeightData,starty,starth,endy,endh); 
    
    %% ��Ӧ��ֵ�������
    fitness=CacuFit(path);                               
    [newbestfitness,newbestindex]=min(fitness);     
    if newbestfitness<bestfitness
        bestfitness=newbestfitness;
        bestpath=path(newbestindex,:);
    end 
    BestFitness=[BestFitness;bestfitness];
    
    %% ������Ϣ��
    cfit=100/bestfitness;
    for i=2:PortGrid-1
        pheromone(i,bestpath(i*2-1),bestpath(i*2))=(1-rou)* ...
            pheromone(i,bestpath(i*2-1),bestpath(i*2))+rou*cfit;
    end
%   display([num2str(kk),'bestpath=[',num2str(bestpath),'],bestfitness=',num2str(bestfitness)])
end
bestfitness
toc
disp(['����ʱ��: ',num2str(toc)]); 
%% ���·��
for i=1:21
    a(i,1)=bestpath(i*2-1);
    a(i,2)=bestpath(i*2);
end
%% ����ͼ
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
view(0,90)%'����ͼ'
hold on
%% 3D��ͼ
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
%% ȥ��3D�ռ�ģ�͵�����ͼ
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

%% ��Ӧ�ȱ仯
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

%% ���ݳ�ʼ��
tic
%��������
load  HeightData
%���񻮷�
LevelGrid=10;%
PortGrid=21;

%����յ������ 
starty=10;starth=4;
endy=8;endh=5;

%�㷨����
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
%% ����·�����ж��Ƿ�Ϸ�
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
    
%% ��ţ�뽨ģ1
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
    %������Ӧ��
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
disp(['����ʱ��: ',num2str(toc)]); 
%% ���·��
%TODO:�޸���Ӧ�ȱ仯�Ļ�ͼ���ִ���
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

%% ��Ӧ�ȱ仯
figure(4)
plot(BestFitness,'LineWidth',2,'Color',[1 0 0]);
hold on

%% ACO-BAS
tic
%���񻮷�
LevelGrid=10;%
PortGrid=21;

%����յ������ 
starty=10;starth=4;
endy=8;endh=5;

%��ʼ��Ϣ��
pheromone=ones(21,21,21);
PopNumber=10; %��Ⱥ����

%�㷨����
bestfitness=inf;
BestFitness=[]; 

eta = 0.9999;%������˥����
eta0=0.998;
c=2;      

step=4;     
step0=21;
n=500;     %iterations
k=PortGrid;  %ά������Ҫ�ߵĲ���
z=1;

%% ��ʼ����·��
[path,pheromone]=searchpath(PopNumber,LevelGrid,PortGrid,pheromone, ...
    HeightData,starty,starth,endy,endh); 
fitness=CacuFit(path);                          %��Ӧ�ȼ���
[bestfitness,bestindex]=min(fitness);           %�����Ӧ��
bestpath=path(bestindex,:);                     %���·��
BestFitness=[BestFitness;bestfitness];          %��Ӧ��ֵ��¼
path1=bestpath';
path=reshape(path1,1,42);
bestpath=path;
bestfitness_store=bestfitness;
% path_store=[0;path;bestpath];



for ii=1:n
%% ��ţ�뽨ģ1
    d0 =step/c;

    while(1)
        dir=rands(PortGrid*2,1);   %��ķ�����Ϊ������21�㣬������������Ϊ21ά��y��h�����������42ά
        dir=(dir/norm(dir))';       %��һ��    ���������ϵ�ά��ȡֵ��[-1,1]����ʮ����������Ϊ����
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
	%˳��·�����޳���Ч·����
for iii=2:20
    if ((path(2*iii+1)-path(2*iii-1))^2 >=(step0)^2)
        path(2*iii-1)=path(2*(iii-1)-1);
        path(2*iii+1)=path(2*(iii-1)+1);
    end
end              
        
    %������Ӧ��
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
disp(['����ʱ��: ',num2str(toc)]); 
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

%% ��Ӧ�ȱ仯
figure(4)
plot(BestFitness,'LineWidth',2,'Color',[0.5 0.5 0.5]);