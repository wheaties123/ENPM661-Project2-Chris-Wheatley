% Chris Wheatley
% ENPM661 Spring 2020
% Project #2

% This script will explore an action space, starting from a user-specified
%   location all the way until the goal node is explored.  Then the optimal
%   path will be drawn.  This script works for both point and rigid robots.

%close all; 
clear all;

% Solicit input node configuration from user
fprintf('\n');
prompt = 'Enter the x,y STARTING node location (e.g. [0,0]), relative to bottom left corner of action space: ';
start_node = input(prompt);
fprintf('\n');

% Solicit goal node configuration from user
fprintf('\n');
prompt1 = 'Enter the x,y GOAL node location (e.g. [181,94]), relative to bottom left corner of action space: ';
goal_node = input(prompt1);
fprintf('\n');

% Solicit robot radius from user
fprintf('\n');
prompt2 = 'Enter robot radius (unit length): ';
r = input(prompt2);
fprintf('\n');

% Solicit obstacle clearance from user
fprintf('\n');
prompt3 = 'Enter obstacle clearance (unit length): ';
c = input(prompt3);
fprintf('\n');

xmax=300; ymax=200;
fig=figure; hold on; axis equal;
% Define black border around action movement area
line([0 xmax xmax 0 0],[0 0 ymax ymax 0],'Color','black');
grid on;
xlabel('X Coordinates'); ylabel('Y Coordinates');
title('Dijkstra Point (Pink Circle = Start ; Pink Triangle = Goal)');

xl = [0 xmax];
yl = [0 ymax];
% Initialize greyed-out action space
patch([xl fliplr(xl)], [yl(1) yl(1) yl(2) yl(2)], [0.5 0.5 0.5]);

% Define obstacle space
% Concave polygon
h1=line([20 50 75 100 75 25 20],[120 150 120 150 185 185 120],'Color','blue');

% Rectangle
h2=line([95 100 35 30 95],[30 38.6603 76.1603 67.5 30],'Color','blue');

% Diamond
h3=line([225 250 225 200 225],[10 25 40 25 10],'Color','blue');

% Ellipse
a=40; % horizontal radius
b=20; % vertical radius
x0=150; % x0,y0 ellipse centre coordinates
y0=100;
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
h4=plot(x,y,'Color','blue');

% Circle 
xunit = 25 * cos(t) + 225;
yunit = 25 * sin(t) + 150;
h5=plot(xunit, yunit,'Color','blue');

% xunit = 10 * cos(t) + 25;
% yunit = 10 * sin(t) + 25;
% h6=plot(xunit, yunit,'Color','blue');

Obstacles=[h1 h2 h3 h4 h5];

startInObstacle = obstacleCheckRigid(Obstacles,start_node,r,c);
goalInObstacle = obstacleCheckRigid(Obstacles,goal_node,r,c);

if startInObstacle==1
    outside_obs_start=1;
    while outside_obs_start==1
        % Display message if start node falls outside of action space 
        fprintf('\n');
        disp('INAVLID START NODE LOCATION! Interference w/Obstacle.');
        fprintf('\n');
        prompt = 'Enter new x,y STARTING node location (e.g. [0,0]), relative to bottom left corner of action space: ';
        start_node = input(prompt);
        outside_obs_start = obstacleCheckRigid(Obstacles,start_node,r,c);
    end
end

if goalInObstacle==1
    outside_obs_goal=1;
    while outside_obs_goal==1
        % Display message if goal node falls outside of action space 
        fprintf('\n');
        disp('INAVLID GOAL NODE LOCATION! Interference w/Obstacle.');
        fprintf('\n');
        prompt = 'Enter new x,y GOAL node location (e.g. [0,0]), relative to bottom left corner of action space: ';
        goal_node = input(prompt);
        outside_obs_goal = obstacleCheckRigid(Obstacles,goal_node,r,c);
    end
end

% Start program run timer
tic

% Plot start and end point
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');
drawnow

uistack(fig,'top');

% Initialize start node info
Nodes(1).x=start_node(1);
Nodes(1).y=start_node(2);
Nodes(1).ParentID=0;
Nodes(1).ID=1;
Nodes(1).Cost2Come=0;
Nodes(1).Explored=1;

goal_node_explored=0;
i=1;
ParentIdx=1;

move_cost_slant=sqrt(2);
move_cost_straight=1;

% While goal node HAS NOT BEEN EXPLORED, perform every possible action and
%   record info in running structure
while goal_node_explored==0
    lowest_cost_path_used=0;
    parent_x=Nodes(i).x; 
    parent_y=Nodes(i).y;
    x_vals=[Nodes.x]; y_vals=[Nodes.y];
    
    % Perform all possible actions for node located in bottom left of grid
    if and(Nodes(i).x==0,Nodes(i).y==0)==1
        
        [x,y] = ActionMoveUp(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')             
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveRight(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')             
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
    % Perform all possible actions for node located in top left of grid
    elseif and(Nodes(i).x==0,Nodes(i).y==ymax)==1
        
        [x,y] = ActionMoveRight(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;            
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')             
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveDown(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else        
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;            
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')             
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
    % Perform all possible actions for node located in bottom right of grid
    elseif and(Nodes(i).x==xmax,Nodes(i).y==0)==1
       
        [x,y] = ActionMoveUp(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')             
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveLeft(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')            
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
    % Perform all possible actions for node located in top right of grid
    elseif and(Nodes(i).x==xmax,Nodes(i).y==ymax)==1
        [x,y] = ActionMoveLeft(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')            
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveDown(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')           
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end        
    
    % Perform all possible actions for node located on left wall of grid
    elseif Nodes(i).x==0    
        [x,y] = ActionMoveUp(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')          
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveRight(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')            
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveDown(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')           
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end        
        
    % Perform all possible actions for node located in on right wall of
    %   grid
    elseif Nodes(i).x==xmax        
        [x,y] = ActionMoveUp(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')         
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveLeft(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')          
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveDown(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')         
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
    % Perform all possible actions for node located on bottom/floor of grid
    elseif Nodes(i).y==0   
        [x,y] = ActionMoveUp(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')         
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveRight(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')          
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveLeft(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')         
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end        
        
    % Perform all possible actions for node located on top/ceiling of grid
    elseif Nodes(i).y==ymax       
        [x,y] = ActionMoveRight(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')       
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveDown(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')          
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveLeft(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')          
            drawnow;
        else
        end
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end        
        
    % Perform all possible actions for node located in part of grid where
    %   walls do not interfere
    else
        [x,y] = ActionMoveRight(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')        
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveDown(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')           
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveLeft(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')            
            drawnow;
        else
        end
        end
        
        [x,y] = ActionMoveUp(parent_x,parent_y);
        x_mask=x_vals==x; y_mask=y_vals==y;
         
        if sum(and(x_mask,y_mask))>=1
        else
        outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
        if outside_obs_goal==0
            lowest_cost_path_used=1;
            ParentIdx=ParentIdx+1;
            Nodes(ParentIdx).x=x;
            Nodes(ParentIdx).y=y;
            Nodes(ParentIdx).ParentID=i;
            Nodes(ParentIdx).ID=ParentIdx;
            Nodes(ParentIdx).Explored=1;
            Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_straight;
            if and(goal_node(1)==x,goal_node(2)==y)==1
                goal_node_explored=1;
            else
            end
            plot(x,y,'ys','MarkerFaceColor','w')           
            drawnow;
        else
        end        
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveUpLeft(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end
        
        if lowest_cost_path_used==0
            [x,y] = ActionMoveDownRight(parent_x,parent_y);
            x_mask=x_vals==x; y_mask=y_vals==y;
             
            if sum(and(x_mask,y_mask))>=1
            else

            outside_obs_goal = obstacleCheckRigid(Obstacles,[x,y],r,c);
            if outside_obs_goal==0
                ParentIdx=ParentIdx+1;
                Nodes(ParentIdx).x=x;
                Nodes(ParentIdx).y=y;
                Nodes(ParentIdx).ParentID=i;
                Nodes(ParentIdx).ID=ParentIdx;
                Nodes(ParentIdx).Explored=1;
                Nodes(ParentIdx).Cost2Come=Nodes(i).Cost2Come+move_cost_slant;
                if and(goal_node(1)==x,goal_node(2)==y)==1
                    goal_node_explored=1;
                else
                end
                plot(x,y,'ys','MarkerFaceColor','w') 
                drawnow
            else
            end
            end
        else
        end 
        
    end
    i=i+1;
    %
end


IDs=cell2mat({Nodes.ID});
PAR_IDs=cell2mat({Nodes.ParentID});
Xs=cell2mat({Nodes.x});
Ys=cell2mat({Nodes.y});
Costs=cell2mat({Nodes.Cost2Come});

k=1;
id(k)=IDs(and(Xs==goal_node(1),Ys==goal_node(2)));
parent(k)=PAR_IDs(and(Xs==goal_node(1),Ys==goal_node(2)));
cost(k)=Costs(and(Xs==goal_node(1),Ys==goal_node(2)));
xval(k)=Xs(and(Xs==goal_node(1),Ys==goal_node(2)));
yval(k)=Ys(and(Xs==goal_node(1),Ys==goal_node(2)));

% Backtrack to find optimal path
backTrackFinished=0;
while backTrackFinished==0
    k=k+1;
    id(k)=parent(k-1);
    parent(k)=PAR_IDs(IDs==id(k));
    cost(k)=Costs(IDs==id(k));
    xval(k)=Xs(IDs==id(k));
    yval(k)=Ys(IDs==id(k));
    if and(xval(k)==start_node(1),yval(k)==start_node(2))==1
        backTrackFinished=1;
    else
    end
end

% Plot optimal path and replot start and end nodes
plot(xval,yval,'g-','LineWidth',1);
drawnow
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');
drawnow
uistack(fig,'top');

% End program run timer
toc
