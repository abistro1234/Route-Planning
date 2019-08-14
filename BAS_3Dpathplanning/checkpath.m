function flag=checkpath(path,HeightData)
    %path 是一个42维的数组，存着y坐标和对应的h，我们用ii来循环，ii表示的是x
    %第二个位置是3，2*ii-1，倒数第二个位置是39，2*ii-1
    flag=1;
    for ii=2:20
        %如果合法的话，这时候的HeightData数值应该低于2*ii*200
        if ((path(2*ii-1)>=20) || path(2*ii-1)<=0 || path(2*ii)>10 || path(2*ii)<=0)
            flag=0;
        elseif HeightData(path(2*ii-1),ii) > 200*path(2*ii)
                flag=0;
            end
        end
    end

