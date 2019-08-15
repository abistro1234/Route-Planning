% function [pathresult,time,pathLength] = test_dynamic(start,goal,mapname)
% clearvars -except start goal mapname;
clear;
close all;
isdynamic = true;% if dynamic = true obstacle move
isdynamicdisplay = false;% if dynamic = true obstacle move
display = false; % display of path plan

[mapname,start,goal]=initpar; % 批量测试时候参数从loop函数中传递
safedist = 5;
% map=im2bw(imread(mapname)); % input map read from a bmp file. for new maps write the file name here
map=im2bw(imread(mapname)); % input map read from a bmp file. for new maps write the file name here
map = 1*map;% 数值化矩阵
% map = map';
[ncols,nrows]=size(map);

%example:map=500*100 logical ncols = 500,nrows = 100
%newmap = safemap(map,safedist);
step = 15;  % size of each step of the BAS
threshold = 15; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 50000;
figure;imshow(map.');axis ([0 ncols 0 nrows]);
% axis xy;
axis on;xlabel ('x');ylabel ('y');
hold on;
plot (start(1), start(2), 'b.', 'MarkerSize', 25);
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);


%% move obstacle   第一张图的移动障碍物参数  对应dynamic_env2
% mox = 400;
% moy = 175;
% mnum = 0;
% totalmovedist = 20;
% movedist = 5;
% mox2=400;moy2=225;        
% tempx = mox;tempy = moy;
% tempx2 = mox2;tempy2 = moy2;
% obswid = 80;
% obslen = 20;
% loopnum=1;


% %% move obstacle   第二张图的移动障碍物参数 对应dynamic_env3
% mox = 200;
% moy = 50;
% mnum = 0;
% totalmovedist = 20;
% movedist = 5;
% mox2=325;moy2=150;        
% tempx = mox;tempy = moy;
% tempx2 = mox2;tempy2 = moy2;
% obswid = 20;
% obslen = 50;
% loopnum=1;


% %% move obstacle   第二张图的移动障碍物参数 对应dynamic_env3
% mox = 200;
% moy = 50;
% mnum = 0;
% totalmovedist = 20;
% movedist = 5;
% mox2=325;moy2=150;        
% tempx = mox;tempy = moy;
% tempx2 = mox2;tempy2 = moy2;
% obswid = 20;
% obslen = 20;
% loopnum=1;
%% move obstacle   第二张图的移动障碍物参数 对应dynamic_env3
mox = 300;
moy = 100;
mnum = 0;
totalmovedist = 20;
movedist = 5;
mox2=300;moy2=350;        
tempx = mox;tempy = moy;
tempx2 = mox2;tempy2 = moy2;
obswid = 20;
obslen = 80;
loopnum=1;

% writerObj=VideoWriter('test.avi'); %figure录制视频
%% 
tic;
[path,iteration,map] = BAS(map,start,goal,step,maxFailedAttempts,display,isdynamic,isdynamicdisplay);
% 平滑直接进行轨迹追踪
% 写个判定。如果平滑后的路径点位于障碍物内，使用原本的对应路径点进行替换
smoothPath = Spline_Smooth(path(:,1), path(:,2), round(length(path(:,1))/4),3);
smoothPath = smoothPath';
toc;
time = toc;
pathLength = 0;
for i=1:length(path)-1, pathLength = pathLength + distanceCost(path(i,1:2),path(i+1,1:2)); end % calculate path length
spathLength = 0;
for i=1:length(smoothPath)-1, spathLength = spathLength + distanceCost(smoothPath(i,1:2),smoothPath(i+1,1:2)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \nSmooth Path Length=%d \n', time, pathLength,spathLength); 
%% 动态显示
if isdynamicdisplay,
figure;
imshow(map.');
axis ([0 ncols 0 nrows]);
% axis xy;
axis on;
% imshow(~obstacle);
hold on;
plot (start(1), start(2), 'b.', 'MarkerSize', 25);
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
% 平滑曲线
a = path(:,1)';  %横坐标
b = path(:,2)'; %纵坐标
p1 = plot(a,b, 'r-o');
a1 = smoothPath(:,1);  %横坐标
b1 = smoothPath(:,2); %纵坐标
p2 = plot(a1,b1, 'b-o');
md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%需要有position
md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%需要有position
pause(0.1)
end
%% 路径追踪
if isdynamic
    baknum=1;%当未规划出道路时后退
    pathresult = start;
    path = smoothPath;% 平滑后路径先存储一下
    start = round(path(2,:));
    while(norm(start-goal)>step*1.5)
        % 先更新环境
        loopnum=loopnum+1;        
        [map,mnum,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2] = dynamic_env3(obswid,obslen,map,loopnum,movedist,totalmovedist,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2,isdynamicdisplay);
        %如果有新的路径就取下一个路径点，如果没有就原地等待。
        
        % 如果没有移动障碍物就使用平滑后的路径
        if ~isempty(existDynamic(map,start)),
            maxFailedAttempts = 5000;
            [path,iteration,map] = BASdynamic(map,start,goal,step,maxFailedAttempts,display,isdynamic,isdynamicdisplay);  
        else
            path = path(2:end,:);
        end
        if length(path(:,1))==1,
            start = pathresult(end-baknum,:);
            baknum=baknum+2;
        else
            start = round(path(2,:));
        end
        % 路径点移动太小不采纳
        if norm(start-pathresult(end,:))>(step*0.9),
            pathresult = [pathresult;start];
        else
            pathresult = [pathresult;pathresult(end,:)];
        end
%         start = round(path(2,:));
        
%         smoothPath = Spline_Smooth(path(:,1), path(:,2), round(length(path(:,1))/4),3);
%         smoothPath = smoothPath';
            % 中间过程 测试时候省略过程
            if isdynamicdisplay,
            delete(md1);
            delete(md2);
            md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%需要有position
            md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%需要有position
            delete(p1);
            delete(p2);
            plot (start(1), start(2), 'b.', 'MarkerSize', 25);
            % 平滑曲线
            a = path(:,1)';  %横坐标
            b = path(:,2)'; %纵坐标
            p1 = plot(a,b, 'r-o');


    %         a1 = smoothPath(:,1);  %横坐标
    %         b1 = smoothPath(:,2); %纵坐标
    %         p2 = plot(a1,b1, 'b-o');
            pause(0.1);
        end
    end
    pathresult = [pathresult;path];
    time = toc;
end


figure;
imshow(map.');
axis ([0 ncols 0 nrows]);
% axis xy;
axis on;
% imshow(~obstacle);
hold on;
            md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%需要有position
            md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%需要有position
% plot (start(1), start(2), 'b.', 'MarkerSize', 25);
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
% 平滑曲线
a = path(:,1)';  %横坐标
b = path(:,2)'; %纵坐标
plot(a,b, 'r-o');
% a1 = smoothPath(:,1);  %横坐标
% b1 = smoothPath(:,2); %纵坐标
a1 = pathresult(:,1);  %横坐标
b1 = pathresult(:,2); %纵坐标
plot(a1,b1, 'b-o');
title ('final path');
% print('-deps', 'finalPath');% 后期论文需要可以直接保存为eps文件 
print('-dbmp', 'finalPath');% 后期论文需要可以直接保存为eps文件 
% end
pathLength = 0;
for i=1:length(pathresult)-1, pathLength = pathLength + distanceCost(pathresult(i,1:2),pathresult(i+1,1:2)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d ', time, pathLength); 
% end