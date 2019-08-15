function [map,mnum,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2] = dynamic_env3(obswid,obslen,map,loopnum,movedist,totalmovedist,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2,isdynamicdisplay)

        mnum = loopnum;%移动次数
        mmod = fix(mnum/totalmovedist);%移动了多少趟;
        mpos = mod(mnum,totalmovedist);%移动障碍物的位置
        %20是移动障碍物大小
        if(mod(mmod,2)==0)
            temp = mpos*movedist;
            y=moy+temp;
            %             map(x-movedist:x+movedist,moy:moy+20)=1;%标为空白
            map(tempx:tempx+obswid,tempy:tempy+obslen)=1;%标为空白
%             map(x:x+obswid,moy:moy+obslen)=0;%标为障碍物
            map(mox:mox+obswid,y:y+obslen)=2;%标为障碍物
            tempx = mox;tempy = y;
            y2 = moy2+temp;
            map(tempx2:tempx2+obswid,tempy2:tempy2+obslen)=1;%标为空白
%             map(x2:x2+obswid,moy2:moy2+obslen)=0;%标为障碍物
            map(mox2:mox2+obswid,y2:y2+obslen)=2;%标为障碍物
            tempy2 = y2;tempx2 = mox2;
        else
            temp = mpos*movedist;
            y = moy+totalmovedist*movedist-temp;
            map(tempx:tempx+obswid,tempy:tempy+obslen)=1;%标为空白
            %             map(x-movedist:x+movedist,moy:moy+20)=1;%标为空白
%             map(x:x+obswid,moy:moy+obslen)=0;%标为障碍物
            map(mox:mox+obswid,y:y+obslen)=2;%标为障碍物
            tempx = mox;tempy = y;
            
            y2 = moy2+totalmovedist*movedist-temp;
            map(tempx2:tempx2+obswid,tempy2:tempy2+obslen)=1;%标为空白
%             map(x2:x2+obswid,moy2:moy2+obslen)=0;%标为障碍物
            map(mox2:mox2+obswid,y2:y2+obslen)=2;%标为障碍物
            tempx2 = mox2;tempy2 = y2;
        end
        dyobp = [tempx,tempy,obswid,obslen;tempx2,tempy2,obswid,obslen];
        mnum = mnum+1;
%         if isdynamicdisplay
%             md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%需要有position
%             md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%需要有position
% 
%             a = path(:,1)';  %横坐标
%             b = path(:,2)'; %纵坐标
%             h2 = plot(a,b, 'r-o');
%             pause(0.1)
%             delete(h2);
%         
% %             pause(0.1);
%             delete(md1);
%             delete(md2);
%         end
end