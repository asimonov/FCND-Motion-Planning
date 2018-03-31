# Project Write-up: 3D Motion Planning

---
#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!
Here I will consider the [rubric](https://review.udacity.com/#!/rubrics/1534/view)
points individually and describe how I addressed each point in my implementation.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

These scripts contain a basic planning implementation that includes the class `MotionPlanning`, subclass of `Drone` (defined in `udacidrone`.)
`MotionPlanning` provides basic code to connect to the drone, arm, takeoff, plan a path, follow the planned path to the goal, land, disarm.
`planning_utils.py` provides useful starting code to create a grid from map representation and plan path in a grid using A* algorithm and a
user-defined heuristic function.

`MotionPlanning` class is different from `BackyardFlyer` that we have seen before in a number of ways:

- it has a new `State` defined called `PLANNING`
- `PLANNING` is envoked once `ARMING` is complete
- once `PLANNING` finishes the drone is transitioned to `TAKEOFF`
- to implement `PLANNING` a function called `plan_path` is defined
- inside `plan_path` the following has to happen
    * set target altitude
    * read home position from config file and set it on the drone
    * get current global position
    * convert to local position using `global_to_local()`
    * read map and convert it to grid representation
    * find start position in the grid
    * set goal position in the grid
    * run A* to find a path in the grid
    * prune the path
    * convert path to waypoints
    * send waypoints to simulator (for visualisation)

The provided `colliders.csv` file represents a map around starting location (first line of the file `lat0 37.792480, lon0 -122.397450`),
which happens to be in San-Francisco:

![Map of SF](./misc/gmaps_start.png)

The remaining lines in `colliders.csv` represent center points of obstacles along with their dimensions in 3D:

```
posX,posY,posZ,halfSizeX,halfSizeY,halfSizeZ
-310.2389,-439.2315,85.5,5,5,85.5
-300.2389,-439.2315,85.5,5,5,85.5
...
```

Here's what same area looks like in the simulator:

![Top Down View](./misc/high_up.png)

### Implementing Path Planning Algorithm

#### 1. Set global home position

Here students should read the first line of the csv file,
extract lat0 and lon0 as floating point values and
use the self.set_home_position() method to set global home.
Explain briefly how you accomplished this in your code.


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.






Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd







