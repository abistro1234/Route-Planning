function [path,iteration,map] = BASdynamic(map,start,goal,step,maxFailedAttempts,display,isdynamic,isdynamicdisplay)
isdynamic = false;
isdisplay=false;
%% field map
field = Fieldmap(map,goal);
% %% move obstacle
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
%% parameter setup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ncols,nrows]=size(map);
counter = 0;
step=step;
threshold = step;
goal = goal;

%%
failedAttempts = 0;
nodes = [];%�������е�nodes
path = [];%����·��
d0=0.001;
d1=5;
d=d1;
eta_d=1;
eta_step=1;
n=maxFailedAttempts;%iterations
k=2;%space dimension
nodes(1,:)=start;
path(1,:)=start;
xbest = start;
x=start;
fbest=fun2(xbest,goal,nodes,map,field,step);
fbest_store=fbest;
pathFound = false;
baknum=0;
% display=true;%display bas
if display,imshow(map.');rectangle('position',[1 1 size(map)-1],'edgecolor','k'); axis on;end
% if display,imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); axis on;axis xy;end
% i = baknum;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%iteration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while failedAttempts<=maxFailedAttempts
%     if mod(failedAttempts,50)==0
%         toc
%         tic;
%         figure;
%         imshow(map.');hold on;
%         a = path(:,1)';  %������
%         b = path(:,2)'; %������
%         htemp = plot(a,b, 'r-o');
%     end
    
    %��һ��
%     if baknum>5,
%         path(end,:)=[];
%         baknum=0;
%     end
%     if isdynamic
%         if mod(failedAttempts,movedist)==0
%             [map,mnum,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2] = dynamic_env(obswid,obslen,map,failedAttempts,movedist,totalmovedist,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2,isdynamicdisplay);
%             field = Fieldmap(map,goal);
%         end
%         %% ��̬��ʾ�޸�
% %         if isdynamicdisplay
% %             %�����ƶ��ϰ���Ĳ���
% %             md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%��Ҫ��position
% %             md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%��Ҫ��position
% %             
% %             a = path(:,1)';  %������
% %             b = path(:,2)'; %������
% %             h2 = plot(a,b, 'r-o');
% %             pause(0.001)
% %             delete(h2);
% %             
% %             %             pause(0.1);
% %             delete(md1);
% %             delete(md2);
% %         end
%         
%     end
    if rand < 1
        % �������
        temp = rand(1,k).* size(map);   % random sample
    else
        temp = goal; % sample taken as goal to bias tree generation to goal
    end
    dir = atan2(temp(1)-path(end,1),temp(2)-path(end,2));
    %     xleft = double(int32(xbest(1:2) + d * [sin(dir)  cos(dir)]));
    %     xright = double(int32(xbest(1:2) - d * [sin(dir)  cos(dir)]));
    xleft = double(int32(path(end,:) + d * [sin(dir)  cos(dir)]));
    xright = double(int32(path(end,:) - d * [sin(dir)  cos(dir)]));
    if ~feasiblePoint(xleft,map)==1&&~feasiblePoint(xright,map)==1,%���������ϰ����� failedAttempts++
        baknum = baknum+1;
        failedAttempts = failedAttempts+1;
        continue;
    end
    if ~feasiblePoint(xright,map),
        x = double(int32(path(end,:) + step * [sin(dir)  cos(dir)]));
%         x = reasonableRange(x,ncols,nrows)
    elseif ~feasiblePoint(xleft,map),
        x = double(int32(path(end,:) - step * [sin(dir)  cos(dir)]));
%         x = reasonableRange(x,ncols,nrows)
    else
        fleft=fun2(xleft,goal,nodes,map,field,step);
        fright=fun2(xright,goal,nodes,map,field,step);
        x = double(int32(path(end,:) - step * [sin(dir)  cos(dir)]*sign(fleft-fright)));
%         x = reasonableRange(x,ncols,nrows)
    end
    if ~feasiblePoint(x,map),%��һ��·�������ϰ����� failedAttempts++
         baknum = baknum+1;
        failedAttempts = failedAttempts+1;
        continue;
    end
    %     fleft=fun2(xleft,goal,nodes,map);
    %     fright=fun2(xright,goal,nodes,map);
    % %     x = x+step*dir*sign(fleft-fright);
    %     x = double(int32(path(end,:) - step * [sin(dir)  cos(dir)]*sign(fleft-fright)));
    f=fun2(x,goal,nodes,map,field,step);
    nodes = [nodes;x];
    %%%%%%%%%%%
    %%���׵����Ҳ���·������˺�ʱ����������δ�������
    %if�ж�����
    %     if f<fbest
    xbest=x;
    fbest=f;
    %     end
    %%%%%%%%%%%
    if ~checkPath(path(end,:), xbest, map) % ·�������ߺ��ϰ�����ײ failedAttempts++
        baknum = baknum+1;
        failedAttempts = failedAttempts + 1;
        continue;
    end
    if distanceCost(path(end,:),goal) < threshold,
        path = [path;goal];
        pathFound = true; break;
    end
    %     if distanceCost(xbest,path(end,:)) > threshold,
    %         path = [path;xbest];
    %     end
    %     path = [path;xbest];
    % path�о��뵱ǰ������ĵ�
%       if length(path(:,1))<10,
         [A, I] = min( distanceCost(path(:,1:2),xbest) ,[],1);
         if I>1,path(I+1:end,:)=[];end
%       else
%         [A, I] = min( distanceCost(path(end-9:end,1:2),xbest) ,[],1);
%         if I>(length(path(:,1))-8),path(I+1:end,:)=[];end
%       end
    
%     if  A < threshold,
%         %     if distanceCost(xbest,path(end,:)) > threshold,
%         baknum = baknum+1;
%         failedAttempts = failedAttempts + 1;
%         continue;
%     end
% if display imshow(map.');
    if display,
        a = path(:,1)';  %������
        b = path(:,2)'; %������
        h2 = plot(a,b, 'r-o');
        pause(0.1)
        delete(h2);
%          line([path(end,1);xbest(1)],[path(end,2);xbest(2)]);
%          counter = counter + 1; 
%          M(counter) = getframe; 
     end % Capture movie frame

    path = [path;xbest];
    d=d*eta_d+d0;
    %     l=l*eta_l+l0;
    step=step*eta_step;
    failedAttempts = failedAttempts + 1;

end
% toc;
iteration = failedAttempts;
% if ~pathFound,
%     error('no path found. maximum attempts reached');
% end
end
