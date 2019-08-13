clear
close all
clc

%初始化部分
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
%迭代部分
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
    %%%%%%%%%%%   记录f最小值及其最小的x
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
%数据显示部分
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1),clf(1),
plot(x_store(1,:),x_store(end,:),'r-o','Markersize',1)
hold on,
plot(x_store(1,:),fbest_store,'b-.')
xlabel('iteration')
ylabel('minimum value')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%被优化的函数，这部分需要换用你自己的被优化函数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y=f(x)
y=norm(x);
end

