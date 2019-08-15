clear
close all
clc
start = [100,100;];
goal = [500,300;];
file = 'mapfile/';
% 随意测试
% mapname = ['mapfile/map2.png'];
mapname = ['mapfile/map3.png'];
iterationnum = 99;
loopnum = 0;
% Pathjihe=[];
for i = 1:1
    loopnum = loopnum+1;
    [path,time,pathLength] = test_dynamic(start(loopnum,:),goal(loopnum,:),mapname(loopnum,:));
    routesize = size(path,1)
    po=0;
%     Pathjihe = [path];% 所有路径集合 暂时未保存
%     Spathjihe = [smoothpath];
    shuju = [routesize,time,pathLength];
    movefile('finalPath.bmp',['./dataSave/',num2str(loopnum),'/map',num2str(loopnum),'_0.bmp']);
%     movefile('diedainum.png',['./dataSave/6.4/dd',num2str(emmm),'/map',num2str(emmm),'ddnum0.png']);
    
%     Pathjihe = [];
%     Spathjihe=[];

    for j =1:iterationnum
%         [route,finalbest,finalstepnum] = main(start(1,:),goal(1,:),mapname(1,:))    
%         [route,finalbest,finalstepnum,ntime,rlength] = main(start(emmm,:),goal(emmm,:),mapname(emmm,:));
        [path,time,pathLength] = test_dynamic(start(loopnum,:),goal(loopnum,:),mapname(loopnum,:));
        po = po+1
        %eval(['!rename' , ',diedainum.png' , ',ddnum',num2str(po),'.png']);
        %eval(['!rename' , ',ConfigurationSpace.png' , ',ConfigurationSpace',num2str(po),'.png']);
        movefile('finalPath.bmp',['./dataSave/',num2str(loopnum),'/map',num2str(loopnum),'_',num2str(po),'.bmp']);
%         movefile('diedainum.png',['./dataSave/6.4/dd',num2str(emmm),'/map',num2str(emmm),'ddnum',num2str(po),'.png']);
        routesize = size(path,1)
        %     routejihe = cat(2,routejihe,[route])
        shuju = cat(1,shuju,[routesize,time,pathLength])
    end
    routemin= min(shuju(:,1));
    routemax = max(shuju(:,1));
    routemean = mean(shuju(:,1));
    
    timemin = min(shuju(:,2));
    timemax = max(shuju(:,2));
    timemean = mean(shuju(:,2));
    
    rlmin = min(shuju(:,3));
    rlmax = max(shuju(:,3));
    rlmean = mean(shuju(:,3));
    save(['./dataSave/mat',num2str(loopnum)]);
    
end
fprintf('平均时间=%d \n最长时间=%d \n最短时间=%d \n', timemean, timemax,timemin); 
fprintf('平均路径点=%d \n最长路径点=%d \n最短路径点=%d \n', routemean, routemax,routemin); 
fprintf('平均路径长度=%d \n最长路径长度=%d \n最短路径长度=%d \n', rlmean, rlmax,rlmin); 