function obstacle(x)
Z = peaks(x)';

%zmin = floor(min(Z(:))); 
zmin = 1.2; % 

zmax = ceil(max(Z(:)));

%zinc = (zmax - zmin) / 40;
zinc = (zmax - zmin) / 20; 
zlevs = zmin:zinc:zmax;

contour(Z,zlevs)

zindex = zmin:2:zmax;

contour(Z,zindex,'LineWidth',2)
end
