% function [pathresult,time,pathLength] = test_dynamic(start,goal,mapname)
% clearvars -except start goal mapname;
clear;
close all;
isdynamic = true;% if dynamic = true obstacle move
isdynamicdisplay = false;% if dynamic = true obstacle move
display = false; % display of path plan

[mapname,start,goal]=initpar; % ��������ʱ�������loop�����д���
safedist = 5;
% map=im2bw(imread(mapname)); % input map read from a bmp file. for new maps write the file name here
map=im2bw(imread(mapname)); % input map read from a bmp file. for new maps write the file name here
map = 1*map;% ��ֵ������
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


%% move obstacle   ��һ��ͼ���ƶ��ϰ������  ��Ӧdynamic_env2
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


% %% move obstacle   �ڶ���ͼ���ƶ��ϰ������ ��Ӧdynamic_env3
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


% %% move obstacle   �ڶ���ͼ���ƶ��ϰ������ ��Ӧdynamic_env3
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
%% move obstacle   �ڶ���ͼ���ƶ��ϰ������ ��Ӧdynamic_env3
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

% writerObj=VideoWriter('test.avi'); %figure¼����Ƶ
%% 
tic;
[path,iteration,map] = BAS(map,start,goal,step,maxFailedAttempts,display,isdynamic,isdynamicdisplay);
% ƽ��ֱ�ӽ��й켣׷��
% д���ж������ƽ�����·����λ���ϰ����ڣ�ʹ��ԭ���Ķ�Ӧ·��������滻
smoothPath = Spline_Smooth(path(:,1), path(:,2), round(length(path(:,1))/4),3);
smoothPath = smoothPath';
toc;
time = toc;
pathLength = 0;
for i=1:length(path)-1, pathLength = pathLength + distanceCost(path(i,1:2),path(i+1,1:2)); end % calculate path length
spathLength = 0;
for i=1:length(smoothPath)-1, spathLength = spathLength + distanceCost(smoothPath(i,1:2),smoothPath(i+1,1:2)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \nSmooth Path Length=%d \n', time, pathLength,spathLength); 
%% ��̬��ʾ
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
% ƽ������
a = path(:,1)';  %������
b = path(:,2)'; %������
p1 = plot(a,b, 'r-o');
a1 = smoothPath(:,1);  %������
b1 = smoothPath(:,2); %������
p2 = plot(a1,b1, 'b-o');
md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%��Ҫ��position
md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%��Ҫ��position
pause(0.1)
end
%% ·��׷��
if isdynamic
    baknum=1;%��δ�滮����·ʱ����
    pathresult = start;
    path = smoothPath;% ƽ����·���ȴ洢һ��
    start = round(path(2,:));
    while(norm(start-goal)>step*1.5)
        % �ȸ��»���
        loopnum=loopnum+1;        
        [map,mnum,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2] = dynamic_env3(obswid,obslen,map,loopnum,movedist,totalmovedist,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2,isdynamicdisplay);
        %������µ�·����ȡ��һ��·���㣬���û�о�ԭ�صȴ���
        
        % ���û���ƶ��ϰ����ʹ��ƽ�����·��
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
        % ·�����ƶ�̫С������
        if norm(start-pathresult(end,:))>(step*0.9),
            pathresult = [pathresult;start];
        else
            pathresult = [pathresult;pathresult(end,:)];
        end
%         start = round(path(2,:));
        
%         smoothPath = Spline_Smooth(path(:,1), path(:,2), round(length(path(:,1))/4),3);
%         smoothPath = smoothPath';
            % �м���� ����ʱ��ʡ�Թ���
            if isdynamicdisplay,
            delete(md1);
            delete(md2);
            md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%��Ҫ��position
            md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%��Ҫ��position
            delete(p1);
            delete(p2);
            plot (start(1), start(2), 'b.', 'MarkerSize', 25);
            % ƽ������
            a = path(:,1)';  %������
            b = path(:,2)'; %������
            p1 = plot(a,b, 'r-o');


    %         a1 = smoothPath(:,1);  %������
    %         b1 = smoothPath(:,2); %������
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
            md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%��Ҫ��position
            md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%��Ҫ��position
% plot (start(1), start(2), 'b.', 'MarkerSize', 25);
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
% ƽ������
a = path(:,1)';  %������
b = path(:,2)'; %������
plot(a,b, 'r-o');
% a1 = smoothPath(:,1);  %������
% b1 = smoothPath(:,2); %������
a1 = pathresult(:,1);  %������
b1 = pathresult(:,2); %������
plot(a1,b1, 'b-o');
title ('final path');
% print('-deps', 'finalPath');% ����������Ҫ����ֱ�ӱ���Ϊeps�ļ� 
print('-dbmp', 'finalPath');% ����������Ҫ����ֱ�ӱ���Ϊeps�ļ� 
% end
pathLength = 0;
for i=1:length(pathresult)-1, pathLength = pathLength + distanceCost(pathresult(i,1:2),pathresult(i+1,1:2)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d ', time, pathLength); 
% end