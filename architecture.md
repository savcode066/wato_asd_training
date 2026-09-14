Costmap: A ROSnode that takes in LaserScans from the /lidar topic, and converts them into a costmap. This is a discretized grid of squares that represent the chances as object exists in that grid with an arbitrary score.

Map Memory: A ROSnode that take in the costmaps overtime and stitches them together to form a global map. This utilizes the robot's own position in the map found as an Odometry message in the '/odom/filtered' topic.

Planner: A ROSnode that takes in the global map and odometry to use the A* algorithm to plan how to get from point A to point B.

Control: A ROSnode that takes in the path from the planner and odometry, and does Pure Pursuit control to physically move the robot along that path. It does this by publishing Twist messages to the '/cmd_vel' topic.