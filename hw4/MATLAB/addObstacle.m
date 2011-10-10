function [ xRange, yRange ] = addObstacle( obstacle, res )
    x = obstacle(1);
    y = obstacle(2);
    w = obstacle(3);
    h = obstacle(4);
    xRange = 1/res * [x, x+w];
    yRange = 1/res * [y, y+h];
end

