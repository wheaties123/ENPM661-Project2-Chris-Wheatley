function [inObstacle] = obstacleCheckPoint(Obstacles,node)
    for i=1:1:length(Obstacles)
        h=Obstacles(i);
        In = inpolygon(node(1),node(2),h.XData,h.YData);
        checker(i)=In;
    end
    if sum(checker)>=1
        inObstacle=1;
    else
        inObstacle=0;
    end
end

