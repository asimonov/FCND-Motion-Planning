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

The provided `colliders.csv` file represents a map of area
around central San-Francisco with obstacle positions given in relation
to global position specified in the first line of 
the file `lat0 37.792480, lon0 -122.397450`:

![Map of SF](./misc/gmaps_start.png)

The remaining lines in `colliders.csv` represent center points of obstacles 
along with their dimensions in 3D:

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

```python
        filename = 'colliders.csv'
        with open(filename) as f:
            for line in f:
                break
        (_, lat0, _, lon0) = line.split()
        lat0 = float(lat0.strip(','))
        lon0 = float(lon0.strip(','))

        # set home position to center of the map
        self.set_home_position(lon0, lat0, 0)
```

This obviously means that the drone better be around that position,
either in the simulation or in the real world.
Because the map only covers certain area around this position.
If the actual drone is started in Easter Island it will have real trouble
planning a path as the map does not go there.
In our case simulator initialises the drone in pretty much the position we set as home.


#### 2. Set your current local position

Having defined global home we can operate in the NED frame centered at the home position.
`global_to_local` function from `frame_utils.py` can convert between global and NED frames.

Under the hood `global_to_local()` uses python `utm` package (Bidirectional UTM-WGS84 converter,
UTM being [Universal Transverse Mercator](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system)).
It is instructive to see how this is done:

```python
def global_to_local(global_position, global_home):
    """
    Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position.

    Returns:
        numpy array of the local position [north, east, down]
    """
    (east_home, north_home, _, _) = utm.from_latlon(global_home[1], global_home[0])
    (east, north, _, _) = utm.from_latlon(global_position[1], global_position[0])

    local_position = np.array([north - north_home, east - east_home, -global_position[2]])
    return local_position
```

And we use this as follows:

```python
        # retrieve current global position of the Drone. (lon, lat, alt) as np.array
        # convert current global coordinates to NED frame (centered at home)
        l_pos = global_to_local(self.global_position, self.global_home)
```




#### 3. Set grid start position from local position

Here we convert local position in NED coordinates, i.e. meters from 
home in Norht, East, Down directions, into
grid coordinates. The grid cell size is 1 meter, from the way we 
defined `create_grid` function.
So the translation from NED to grid is straightforward - just round NED 
coordinates to the nearest integer.

#### 4. Set grid goal position from geodetic coords

I have defined `g_goal` variable in global coordinates and pass it into plan_path function.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

At first I have modified A* by implementing extra Actions to move north-east, north-west,
south-west, south-east with cost of `sqrt(2)`.
Using collinearity check the path is down from 451 waypoints to just 22.
Here is what the pruned path looked:

![A* with diagonal moves and collinearity pruning](./misc/a_star_coll.png)

Immediately I noticed that the time taken to plan such a path was 7.98 seconds.
So I started to tune the algorithm.
I have profiled `a_star` code using [`cprofilev`](https://github.com/ymichael/cprofilev), an easy python profiling tool.
It turned out that heuristic function was taking most of the time.
`np.linalg.norm` was the culprit, so I replaced it with `sqrt(x**2+y**2)`, which cut the time to 3.38 secs.
The next biggest time consumer is get property `delta` on `Action`.
Replacing it with 2 simple functions to get first and second elements cuts time to 2.98 secs.
Changing `Action.cost` from being a property to simple function cuts time further to 2.9 sec.
The resulting code is less generic and specific to 2D planning, but the performance improvements are well worth it!
Unfortunately with grid based approach to planning there are a lot of computations to perform, so I have left it there.

Then I replaced collinearity pruning by Bresenham ray tracing algorithm
to prune not just straigh line path segments, but segments that
can 'short-cut' parts of the path using straight line free-space segments.
The path looked much better, without any performance issues:

![A* with Bresenham ray-tracing pruning](./misc/a_star_bresenham.png)

The path looks 'smoother' than simple collinearity pruning, but does go very close to the buildings.
So next I tried a grid/graph hybrid approach.
I used grid to Voronoi graph transformation shown in the lectures, using center
points of the obstacles as seeds for Voronoi space segmentation. With subsequent ray-tracing tests to prune
resulting Voronoi ridges that collide with obstacles.
The current map representation generates ~9700 Voronoi ridges that need to be mutually tested by Bresenham if they
can connect through the free space. Only ~1400 ridges satisfy this condition. But because of this step the planning
takes a lot longer than simple A* on grid.
The resulting path is much safer as it clears obstacles by a wide margin:

![A* on Voronoi-inspired graph](./misc/a_star_graph.png)


#### 6. Cull waypoints 

See above. I tried both collinearity and ray-tracing for grid-based A* search.
And collinearity for A* on graph. It does result in small reduction of graph-based path length.

### Execute the flight
#### 1. Does it work?

The flight works beautifully.

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







