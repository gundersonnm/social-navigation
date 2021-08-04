# social-navigation
A life-long learning framework to enable the safe operation of a mobile robot in human-populated, dynamic environments.

SETUP INSTRUCTIONS:

RoboSim1 - RoboSim5:
 1) Open your preferred code editor (Atom, Vim, BlueJ, etc.) to run the code. 
 2) In terminal, install pygame.

RoboSim6 - RoboSim7:
 1) Open your preferred code editor (Atom, Vim, BlueJ, etc.) to run the code.
 2) In terminal, install shapely.


ALGORITHM FRAMEWORK:

STEP 1: Environment Initialization
- Set initial and final coordinate for robot and final destination
- ‘Draw’ direct, straight line from start to end point coordinates
- In complete map space (entire area between start point and final end destination), overlay node grid. These will be the points the robot will travel between. 

STEP 2: Ideal Static End to End Path Planning
 - From the current node, choose 10 closest nodes around it - independent of any factors (including obstacles). Add these chosen nodes to a list to have ready.
 - Disqualification
  -  From the chosen node list, node paths will be firstly disqualified if they are behind the current node. This promotes forward motion. Disqualify these nodes from the chosen node list.
  - From that list, disqualify any nodes that create an angle greater than 45 degrees from the straight start to end path line. This promotes more direct forward motion. Disqualify these nodes from the chosen node list.
  - From that list, check if there are any node angles smaller than 20 degrees. If there are, choose the one with the longest distance. This too promotes direct forward motion. 
    - If there are no node angles smaller than 20 degrees, pick the next smallest node angle. 
    - If there are no small node angles the program is allowed to pick angles larger than 45. 
    - If there are none under 45, it is allowed to choose nodes that require backwards motion. 

STEP 3: Static Obstacle Detection and Probability Based Pedestrian Trajectory Vectors
- From an object detection algorithm pulling from octomap 
  - Algorithm detects static obstacles and draws polygons around these obstacles
  - Algorithm detects dynamic obstacles (pedestrians), and will draw both a polygon and a probability based pedestrian trajectory vector
    - Assign theta, x y coordinate position, and velocity value to vector of visible pedestrians
    - If in motion, assume previous motion pattern will continue, if stationary, assume remaining stationary 
    - More information with the creation of more detailed probability vectors is being studied in another lab and will be added into this project later. 

STEP 4: Ideal Path Deviation due to Pedestrians
- If the chosen end to end path intersects with either static obstacles or dynamic pedestrians, it will alter on a priority ranking basis that follows identical rules as the first iteration of disqualifications. 
- If an obstacle is predicted to be in conflict with the path, the surrounding nodes from the initial path plan will be re-inspected. Whichever node was in ‘second place’ to be chosen initially will be re-chosen and checked for intersection with the obstacle. If the obstacle is still predicted to collide, the ‘third place’ node will be checked, and so on. The priority ranking is chosen based on if the node is behind the current node, and the angle of the node path, as well as length of the node path. This effectively snaps the end to end path around each obstacle while maintaining relative linearity and smoothness.
The path deviation will continually update as the robot moves through the end to end path, which will allow for dynamic obstacles to move through the environment without being detrimental to the path plan and informed movement. 

