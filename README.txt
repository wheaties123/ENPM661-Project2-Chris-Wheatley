Chris Wheatley
University of Maryland (College Park)
ENPM661 Spring 2020
Dr. Monfaredi
Project #2

Project Description:
These programs will leverage Dijkstra's algorithm to explore an action space and naviagte obstacles to generate an optimal path, for both point and rigid robots.

The project contains the following files: 
	-Dijkstra_point.m
	-Dijkstra_rigid.m
	-obstacleCheckPoint.m 
	-obstacleCheckRigid.m 
	-ActionMoveRight.m
	-ActionMoveLeft.m
	-ActionMoveUp.m
	-ActionMoveDown.m
	-ActionMoveUpRight.m
	-ActionMoveUpLeft.m
	-ActionMoveDownRight.m
	-ActionMoveDownLeft.m

User Instructions:
1) Ensure that MATLAB is installed on your machine (2017b or later).

2) Download all project contents and unzip the "proj2_chris_wheatley_matlab.zip" contents into a newly created local directory.

3) In MATLAB, change the working directory to the working directory created in step #2.

4) For point robots:
	-Run "Dijkstra_point.m"
	-A prompt will appear asking to enter location of initial node. Enter it between brackets and with column separation (Ex: "[5,5]")
	-Another prompt will appear asking to enter location of goal node. Enter it between brackets and with column separation (Ex: "[5,5]")
	-If there is conflict with these points, the program will prompt you again, so please enter new points.

   For rigid robots:
   -Run "Dijkstra_rigid.m"
	-A prompt will appear asking to enter location of initial node. Enter it between brackets and with column separation (Ex: "[5,5]")
	-Another prompt will appear asking to enter location of goal node. Enter it between brackets and with column separation (Ex: "[5,5]")
	-Another prompt will appear asking to enter the radius of the robot. Enter a number.  This represents a radius of unit length on the grid.
	-Another prompt will appear asking to enter the clearance for the robot. Enter a number.  This represents a radius of unit length on the grid.
	-If there are conflicts with this information, the program will prompt you again, so please enter new points.
   
5) A MATLAB figure (.fig) should appear and be updating in real time, colring in all explored nodes.  Unexplored area in action space is colored grey, explored area is colored yellow, and obstacles are outlined in blue.
6) Once the program finishes running, node exploration should stop, explored nodes should remain colored in yellow, and the optimal path should appear colored in green.
7)   The pink circle is the start point and the pink triangle is the goal point.  If you would like to zoom in, select the + magnifying glass the top ribbon of figure and use your mouse to zoom.

NOTES:
- I am submitting this 1 dat late, but this should be okay since we have 4 late days total for projects 1,2,3, and 4.
- For the rigid robot, some nodes within clearance threshold of an obstacle may be explored, but the final path WILL NOT draw in this area, which is good.
- Though drawing the optimal path takes almost no time at all, the node exploration takes quite a long time. For instance, exploration from [5,5] to [295,195] took me a bit over hours to run on my machine. I have tried relentlessly to fix this but its still pretty slow.  Nodes somewhat close to eachother take reasonably quick.
- I worked individually on this project, with permission from the professor.