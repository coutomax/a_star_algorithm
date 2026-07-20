# Implementation of the A* algorithm
A* is an algorithm designed to find the best path to a goal.

This is a practical demonstration of the A* algorithm for GameMaker that I developed for use in projects and for educational purposes.
This version allows for customization and adjustments to meet the developer's needs.

# Setup
- You can download the .yymps file or just copy the raw script from the .gml file, both works fine. :shipit:

# Features

- Structure based on the struct call within the Events of the object seeking the path.
- Groups functions for runtime visualization and debugging.
- Allows for direct execution of the algorithm as desired.
- Enables step-by-step execution of the algorithm to visualize its operation incrementally.
- Algorithm optimized for multiple executions per frame (I recommend using YYC instead of the VM when comparing performance).

# Availabe Grid Functions
-  `set_grid_properties(_cell_size = 32, _obstacles = [], _exceptions = [])` Sets the properties of the grid used for A* pathfinding.
-  `create_grid()` Called when the grid needs to be created and filled with obstacles.
-  `remove_instance(instance = noone)` Removes an instance from the grid, marking its occupied cells as empty.
-  `draw_grid()` Called on Draw Event. Draws the grid.
-  `draw_coordinates()` Draws the coordinates of each cell in the grid.

# Available A* Functions
-  `a_star(_grid = undefined, _heuristic = manhattan_tie_breaker)` Core of the A* execution.
-  `init()` You don't need to call this one; it simply initializes the structures that are used.
-  `add_instances(_instance = noone, _target = noone)` Adds an instance and its target to the A* algorithm for pathfinding.
-  `grid_init()` Optional method to initialize the grid properties.
-  `change_grid()` Changes the grid properties used by the A* algorithm.
-  `path_finder()` Called every time you want to calculate the path until the target is reached of no path is found.
-  `on_activate()` Called during any event, it is used to track the algorithm's progression step by step. It serves to visualize its operation.
-  `move_instance()` It serves to moving the instance `seeker_object` after the completion of path building process.
-  `print_timers()` Print the execution times, exclusively to measure the completion time.
-  `clear()` Call this if you want to clear the vectors and other structures used.
-  `draw_obstacles()` Draws the obstacles in the grid, highlighting the cells that are occupied by obstacles.
-  `draw_current_node()` Draws the current node in the grid.
-  `draw_path()` Draws the path from the target to the instance in the grid after the path building process.
-  `draw_closed_set()` Draws the closed set in the grid.

- The remaining non-callable functions are properly documented in the code using the JSDoc standard to facilitate understanding.

# Quick usage:

## Create Event:
- In the Create Event, call the script execution and assign it to a variable. `a = a_star(global.grid);`
- Then call the `a.add_instances()` function that trigger internalyy the `init()` setting the base data structures and parameters.
- If you wants the automatic execution just see the  `Step Event` topic.

# Step Event:
- Here you call `a.path_finder()`, which will execute until it reaches the goal or exhausts the possible nodes for doing so.
- You can also use `a.on_activate();` in any event of your choice to execute the algorithm step-by-step, at a slow pace, to visualize it in action.

- Example: (Controller Create Event)
  
```gml
global.grid_properties = set_grid_properties(32, [obj_wall]);

// Can be used to check when the global.grid changes, and to create the grid when the game starts.
global.grid_properties.create_grid(); 
global.grid = variable_clone(global.grid_properties);
```

- Example: (Enemy Create Event)
```gml
move_speed  = 4;

a = a_star(global.grid);
a.add_instances(id, obj_target);
```

- Example: (Enemy Step Event)
```gml
a.path_finder();
a.move_instance(move_speed);
```