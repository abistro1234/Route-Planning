function [map]=safemap(map,safedist)
    [ncols,nrows]=size(map);
    for i = 1:ncols
        for j = 1:nrows
            x = i;
            y=j;
            B = map(x-safedist:x+safedist,y-safedist:y+safedist);
            flag = find(B==0);
            if  ~isempty(flag)
                map(x,y)=0.5;
            end
        end
    end
        
end