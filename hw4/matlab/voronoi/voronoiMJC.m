pts = [];

res = 0.1;

pts = [pts; boxpoints(1,1,19,9,res)];
pts = [pts; boxpoints(3,3,3,3,res)];
pts = [pts; boxpoints(9,5,3,3,res)];
pts = [pts; boxpoints(15,4,3,3,res)];

[vx, vy] = voronoi(pts(:,1),pts(:,2));

voronoiLineset = [vx', vy'];
voronoiLineset = voronoiLineset(:,[1,3,2,4]);
border=         [1,1,1,10;
                 1,10,20,10;
                 20,10,20,1;
                 20,1,1,1];                
obstacle1=      [3,3,6,3;
                 6,3,6,6;
                 6,6,3,6;
                 3,6,3,3];
obstacle2=      [9,5,12,5;
                 12,5,12,8;
                 12,8,9,8;
                 9,8,9,5];
obstacle3=      [15,4,18,4;
                 18,4,18,7;
                 18,7,15,7;
                 15,7,15,4];
checkLines = [border;obstacle1;obstacle2;obstacle3];

mapVx = checkLines(:,[1 3])';
mapVy = checkLines(:,[2 4])';

h = figure(1);

plot(mapVx,mapVy,'k'); hold on;
struct = lineSegmentIntersect(voronoiLineset,checkLines);

for ii = 1:length(struct.intAdjacencyMatrix),
    goodLine = true;
    if isequal(inpolygon(vx(:,ii),vy(:,ii),obstacle1(:,1),obstacle1(:,2)),[1;1]),
        goodLine = false;
    elseif isequal(inpolygon(vx(:,ii),vy(:,ii),obstacle2(:,1),obstacle2(:,2)),[1;1]),
        goodLine = false;
    elseif isequal(inpolygon(vx(:,ii),vy(:,ii),obstacle3(:,1),obstacle3(:,2)),[1;1]),
        goodLine = false;
    end
    if isequal(struct.intAdjacencyMatrix(ii,:),zeros(1,length(checkLines))) && goodLine
        plot(vx(:,ii),vy(:,ii),'r')
    end
end

axis([0,21,0,11]);
axis equal;
grid on;