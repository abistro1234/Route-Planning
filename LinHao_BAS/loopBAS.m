clear
close all
clc
start = [100,100
   ];
goal = [500,300
    ];
file = 'mapfile/';
% ÀÊ“‚≤‚ ‘
% mapname = ['mapfile/map2.png';'mapfile/map2.bmp';'mapfile/map3.bmp';'mapfile/map4.bmp'];
mapname = ['mapfile/map2.png'];
loopnum = 199;
emmm = 0;
for i = 1:2
    emmm = emmm+1;
    [path,smoothPath,time,pathLength,spathLength] = main(start(emmm,:),goal(emmm,:),mapname(emmm,:));
    sizeroute = size(path,1)
    po=0;
    Pathjihe = [path];
%     Spathjihe = [smoothpath];
    shuju = [sizeroute,time,pathLength,spathLength];
    movefile('finalPath.bmp',['./dataSave/',num2str(emmm),'/map',num2str(emmm),'CC0.png']);
%     movefile('diedainum.png',['./dataSave/6.4/dd',num2str(emmm),'/map',num2str(emmm),'ddnum0.png']);
    
    Pathjihe = [];
%     Spathjihe=[];

    for j =1:loopnum
%         [route,finalbest,finalstepnum] = main(start(1,:),goal(1,:),mapname(1,:))    
%         [route,finalbest,finalstepnum,ntime,rlength] = main(start(emmm,:),goal(emmm,:),mapname(emmm,:));
        [path,smoothPath,time,pathLength,spathLength] = main(start(emmm,:),goal(emmm,:),mapname(emmm,:));
        po = po+1
        %eval(['!rename' , ',diedainum.png' , ',ddnum',num2str(po),'.png']);
        %eval(['!rename' , ',ConfigurationSpace.png' , ',ConfigurationSpace',num2str(po),'.png']);
        movefile('finalPath.bmp',['./dataSave/',num2str(emmm),'/map',num2str(emmm),'CC',num2str(po),'.png']);
%         movefile('diedainum.png',['./dataSave/6.4/dd',num2str(emmm),'/map',num2str(emmm),'ddnum',num2str(po),'.png']);
        sizeroute = size(path,1)
        %     routejihe = cat(2,routejihe,[route])
        shuju = cat(1,shuju,[sizeroute,time,pathLength,spathLength])
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
    
    srlmin = min(shuju(:,4));
    srlmax = max(shuju(:,4));
    srlmean = mean(shuju(:,4));
    save(['./dataSave/mat',num2str(emmm)]);
    
end