function [map,mnum,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2] = dynamic_env(obswid,obslen,map,loopnum,movedist,totalmovedist,mox,moy,mox2,moy2,tempx,tempx2,tempy,tempy2,isdynamicdisplay)

        mnum = loopnum;%�ƶ�����
        mmod = fix(mnum/totalmovedist);%�ƶ��˶�����;
        mpos = mod(mnum,totalmovedist);%�ƶ��ϰ����λ��
        %20���ƶ��ϰ����С
        if(mod(mmod,2)==0)
            temp = mpos*movedist;
            x=mox+temp;
            %             map(x-movedist:x+movedist,moy:moy+20)=1;%��Ϊ�հ�
            map(tempx:tempx+obswid,tempy:tempy+obslen)=1;%��Ϊ�հ�
%             map(x:x+obswid,moy:moy+obslen)=0;%��Ϊ�ϰ���
            map(x:x+obswid,moy:moy+obslen)=2;%��Ϊ�ϰ���
            tempx = x;tempy = moy;
            x2 = mox2+temp;
            map(tempx2:tempx2+obswid,tempy2:tempy2+obslen)=1;%��Ϊ�հ�
%             map(x2:x2+obswid,moy2:moy2+obslen)=0;%��Ϊ�ϰ���
            map(x2:x2+obswid,moy2:moy2+obslen)=2;%��Ϊ�ϰ���
            tempx2 = x2;tempy2 = moy2;
        else
            temp = mpos*movedist;
            x = mox+totalmovedist*movedist-temp;
            map(tempx:tempx+obswid,tempy:tempy+obslen)=1;%��Ϊ�հ�
            %             map(x-movedist:x+movedist,moy:moy+20)=1;%��Ϊ�հ�
%             map(x:x+obswid,moy:moy+obslen)=0;%��Ϊ�ϰ���
            map(x:x+obswid,moy:moy+obslen)=0;%��Ϊ�ϰ���
            tempx = x;tempy = moy;
            
            x2 = mox2+totalmovedist*movedist-temp;
            map(tempx2:tempx2+obswid,tempy2:tempy2+obslen)=1;%��Ϊ�հ�
%             map(x2:x2+obswid,moy2:moy2+obslen)=0;%��Ϊ�ϰ���
            map(x2:x2+obswid,moy2:moy2+obslen)=0;%��Ϊ�ϰ���
            tempx2 = x2;tempy2 = moy2;
        end
        dyobp = [tempx,tempy,obswid,obslen;tempx2,tempy2,obswid,obslen];
        mnum = mnum+1;
%         if isdynamicdisplay
%             md1 = rectangle('Position',[tempx,tempy,obswid,obslen]);%��Ҫ��position
%             md2 = rectangle('Position',[tempx2,tempy2,obswid,obslen]);%��Ҫ��position
% 
%             a = path(:,1)';  %������
%             b = path(:,2)'; %������
%             h2 = plot(a,b, 'r-o');
%             pause(0.1)
%             delete(h2);
%         
% %             pause(0.1);
%             delete(md1);
%             delete(md2);
%         end
end