function [ map ] = field_gen( resolution )
    map = zeros(1/resolution * 20, 1/resolution * 10);

    % Add the border
    map(:,1) = 255;
    map(1,:) = 255;
    map(end,:) = 255;
    map(:,end) = 255;

    % Add obstacles
    obstacles = [2,2,3,3;8,5,3,3;15,3,3,3];

    for ii = 1:size(obstacles,1),
        [xrange,yrange] = addObstacle(obstacles(ii,:), resolution);
        map(xrange(1):xrange(2),yrange(1):yrange(2)) = 255;
    end
end