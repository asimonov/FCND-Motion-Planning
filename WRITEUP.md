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

We set global home position to the center of the map, as specified in `colliders.csv`
This is trivial piece of python file/string manipulation code.

#### 2. Set your current local position

Having defined global home we can use NED frame in reference to the home position.
`global_to_local` function from `frame_utils.py` can do this for us, given home position and our current global coordinates
(returned by `Drone.global_position`.)
So the NED frame has its center at the global home position.

#### 3. Set grid start position from local position

Here we convert local position in NED coordinates, i.e. meters from home in Norht, East, Down directions, into
grid coordinates. The grid cell size is 1meter, from the way we defined `create_grid` function.
So the translation from NED to grid is straightforward - just round NED coordinates to the nearest integer.

#### 4. Set grid goal position from geodetic coords

I have defined `g_goal` variable in global coordinates and pass it into plan_path function.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

At first I have modified A* by implementing extra Actions to move north-east, north-west,
south-west, south-east with cost of `sqrt(2)`.

The time taken to plan path has jumped to 7.98 seconds.
So I started to profile `a_star` code using [`cprofilev`](https://github.com/ymichael/cprofilev), an easy python profiling tool.
It turned out that heuristic function was taking most of the time.
`np.linalg.norm` was the culprit, so I replaced it with `sqrt(x**2+y**2)`, which cut the time to 3.38 secs.
The next biggest time consumer is get property `delta` on `Action`.
Replacing it with 2 simple functions to get first and second elements cuts time to 2.98 secs.
Changing `Action.cost` from being a property to simple function cuts time further to 2.9 sec.
The resulting code is less generic and specific to 2D planning, but the performance improvements are well worth it!
Unfortunately with grid based approach to planning there are a lot of computations to perform, so I have left it as is.



!!!
more creative solutions are welcome. Explain the code you used to accomplish this step.


#### 6. Cull waypoints 

Using collinearity check the path is down from 451 waypoints to just 22. Works flawlessly.
!!!

### Execute the flight
#### 1. Does it work?

!!!
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson.
You could try implementing a vehicle model to take dynamic constraints into account, or implement a
replanning method to invoke if you get off course or encounter unexpected obstacles.






Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd







