Costmap Node
Should contain the following ROS constructs:

1 Subscriber that subscribes to the '/lidar' topic for sensor_msgs::msg::LaserScan messages
1 Publisher that publishes nav_msgs::msg::OccupancyGrid messages to a '/costmap' topic


Equivalent to basic pereption, the Costmap Node is responsible for processing data from laserscans to create a discretized map of where the robot can and cannot go. A simple costmap can have binary values 0 and 1 and is called an occupancy grid. However, often sensor readings are noisey, so when a laser scan tells you something is 10m away, it might actually be 9.5-10.5 meters away. That is why a costmap is useful. It allows us to represent the map as a range of values representing the 'cost' that would incur should the robot cross into that part of the map. This means that a high cost equates to places where objects are definitely present and the robot should avoid. Areas of the map with low cost represent places where we are uncertain of whether an object is near, but we should trek safely nontheless.

Steps to Create a Basic Costmap
Subscribe to the /lidar Topic

Receive sensor_msgs::msg::LaserScan messages from the /lidar topic.
The LaserScan message contains:
angle_min, angle_max: The start and end angles of the scan.
angle_increment: The angular resolution of the scan.
ranges: Array of distance measurements.
Initialize the Costmap

Create a 2D array to represent the OccupancyGrid. Each cell corresponds to a grid space in the real world.
Define the resolution of the costmap (e.g., 0.1 meters per cell) and its size.
Initialize all cells to a default value (e.g., 0 for free space).
Convert LaserScan to Grid Coordinates

For each valid range value in ranges:
Compute the Cartesian coordinates of the detected point:

x
=
r
a
n
g
e
⋅
cos
⁡
(
a
n
g
l
e
)
x=range⋅cos(angle)
y
=
r
a
n
g
e
⋅
sin
⁡
(
a
n
g
l
e
)
y=range⋅sin(angle)
Transform these coordinates into grid indices using the resolution and origin of the costmap.

Mark Obstacles

Set the cells corresponding to detected obstacle positions to a high cost (e.g., 100 for "occupied").
Inflate Obstacles

Define an inflation radius (e.g., 1 meter) and a maximum cost for inflated cells.
For each obstacle cell:
Calculate the Euclidean distance to surrounding cells.
Assign a cost to the surrounding cells based on the distance:
c
o
s
t
=
m
a
x
_
c
o
s
t
⋅
(
1
−
d
i
s
t
a
n
c
e
i
n
f
l
a
t
i
o
n
_
r
a
d
i
u
s
)
cost=max_cost⋅(1− 
inflation_radius
distance
​
 )
Only assign a cost if the calculated cost is higher than the cell's current value.
Do not assign a cost to cells beyond the inflation radius.
Publish the Costmap

Convert the 2D costmap array into a nav_msgs::msg::OccupancyGrid message.
Populate the OccupancyGrid fields:
header: Include the frame ID and timestamp.
info: Define the resolution, origin, and size of the grid.
data: Flatten the 2D costmap array into a 1D array.
Publish the message to the /costmap topic.
Code Example (Pseudo-ROS 2 Implementation)
void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}
Key Points
The LaserScan data provides obstacle locations in polar coordinates, which are converted to grid coordinates.
The costmap marks obstacles and inflates costs around them using a linear scale within a defined radius.