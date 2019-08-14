function fitness=cacufit(Z,path,k)
% 只针对peaks(x)的障碍物进行判断
% 如需其他障碍物，请使用multiple_regular_obstacle/cacufit.m
fitness=0;
    for i=2:k
        j=round(path(i));
        if j<=0.1 || j>= k
            obstacle = 100;
        elseif Z(i,j) > 1.2 || (path(i)-path(i-1))>3
            obstacle = 100;
        else
            obstacle = 0;
        end
        fitness = fitness + sqrt((path(i)-path(i-1))^2+1)+obstacle;
    end
end