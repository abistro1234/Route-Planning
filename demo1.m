clear
close all
clc

%��ʼ������
eta=0.95;
c=5;%ratio between step and d0
step=1;%initial step set as the largest input range
n=100;%iterations
k=20;%space dimension
x=rands(k,1);%intial value
xbest=x;
fbest=f(xbest);
fbest_store=fbest;
x_store=[0;x;fbest];
% display(['0:','xbest=[',num2str(xbest'),'],fbest=',num2str(fbest)])
display(['0:','xbest=[',(xbest'),'],fbest=',num2str(fbest)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:n
    d0=step/c;
    dir=rands(k,1);
    dir=dir/(eps+norm(dir));
    xleft=x+dir*d0/2;
    xright=x-dir*d0/2;
    fleft=f(xleft);
    fright=f(xright);
    x=x-step*dir*sign(fleft-fright);
    f1=f(x);
    %%%%%%%%%%%   ��¼f��Сֵ������С��x
    if f1<fbest
        xbest=x;
        fbest=f1;
    end
    %%%%%%%%%%%

    x_store=cat(2,x_store,[i;x;f1]);
    fbest_store=[fbest_store;fbest];
    display([num2str(i),':xbest=[',num2str(xbest'),'],fbest=',num2str(fbest)])
    %%%%%%%%%%%
    step=step*eta;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ʾ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1),clf(1),
plot(x_store(1,:),x_store(end,:),'r-o','Markersize',1)
hold on,
plot(x_store(1,:),fbest_store,'b-.')
xlabel('iteration')
ylabel('minimum value')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���Ż��ĺ������ⲿ����Ҫ�������Լ��ı��Ż�����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y=f(x)
y=norm(x);
end

