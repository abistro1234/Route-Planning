function flag=checkpath(path,HeightData)
    %path ��һ��42ά�����飬����y����Ͷ�Ӧ��h��������ii��ѭ����ii��ʾ����x
    %�ڶ���λ����3��2*ii-1�������ڶ���λ����39��2*ii-1
    flag=1;
    for ii=2:20
        %����Ϸ��Ļ�����ʱ���HeightData��ֵӦ�õ���2*ii*200
        if ((path(2*ii-1)>=20) || path(2*ii-1)<=0 || path(2*ii)>10 || path(2*ii)<=0)
            flag=0;
        elseif HeightData(path(2*ii-1),ii) > 200*path(2*ii)
                flag=0;
            end
        end
    end

