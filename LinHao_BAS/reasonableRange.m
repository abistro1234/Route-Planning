function[re] = reasonableRange(x,ncols,nrows)
theta = x;
x = theta(1);
y = theta(2);
if x>=ncols
    x = ncols-1;
end
if x<1
    x=1;
end
if y>=nrows
    y=nrows-1;
end
if y<1
    y=1;
end
re = [x,y];

end