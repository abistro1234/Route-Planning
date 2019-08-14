%% �ú���������ʾBASӦ����·���滮
    %����ţ�ܵ�����ʱ���൱����һ���ǳ���ķ���

%% ��ջ���
clc
clear
close all%�ر�����ͼ��

%% ���ݳ�ʼ��

x=18;%�߶�Զ
eta=0.99993;
step=4;%����
n=50000;%��������
%p=0.3; %�ϰ�����ײ�ĳͷ�������

ymax=x; %y������,��x��ͬ��Ϊ������
ymin=0;  %y������
Z=peaks(x); %�γ��ϰ���
t=10; %����ͼ����ʾ����
f=5;  %ͼ���֡��

bestfitness=inf; %��Ӧ�����޴�
c=5; %��������ı�����ÿ�θı䲽�����ܸı���ĳ����ˣ�


y=rands(x,1); 

%% ����·��
path = ymin + (ymax-ymin)*rand(1,x); %�������·��
path = roundn(path,-2);%ȡС�������λ
path(1)=x/2; path(x)=x/2;%�̶������յ�
fitness = cacufit(Z,path,x);%������Ӧ��
BestFitness=bestfitness;%�����·�����£���ʵʱ·����


if fitness < bestfitness
    bestpath=path;
    bestfitness=cacufit(Z,bestpath,x);
end
BestPath = bestpath;
display(['0:','bestpath=[',num2str(bestpath),'],bestfitness=',num2str(bestfitness)])

d0 = step/c;
tic
for ii=1:n %
%% ��ţ�뽨ģ1  
    %̽��������ƶ�������������ѭ����ͬʱ����21ά���ƶ���y��z���򣩣���������Լ������
    dir=rands(x,1);   %��ķ�����Ϊ������21�㣬������������Ϊ21ά. 
    dir=dir'/(eps+norm(dir'));       %��һ��
    dir=roundn(dir,-2); dir(1)=0; dir(x)=0;
    leftpath=path+dir*d0/2;
    leftfitness=cacufit(Z,leftpath,x);
    rightpath=path-dir*d0/2;
    rightfitness=cacufit(Z,rightpath,x); 
    path=path-step*dir*sign(leftfitness-rightfitness);
    %������Ӧ��
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

%% ��Ӧ�ȱ仯


figure(1)
hold on
plot(bestpath)
%k=size(BestPath,1);
%plot(BestPath(k,:))
axis([0 x 0 x]);
grid on
title('·���仯ʾ��')
xlabel('�������')
ylabel('�������')
axis equal
obstacle(x)
hold off

bestfitness

save('matPath.mat','BestPath','bestfitness')

%nbmovie(x,t,f,BestPath)



%% ʵ���¼
% �ܳ�������Ϊֹ��õĿ����ƹ��ϰ���Ľ��

