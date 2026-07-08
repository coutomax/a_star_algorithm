# Implementation of the A* algorithm
A* is an algorithm designed to find the best path to a goal.

This is a practical demonstration of the A* algorithm for GameMaker that I developed for use in projects and for educational purposes.
This version allows for customization and adjustments to meet the developer's needs.

# Setup
- You can download the .yymps file or just copy the raw script from the .gml file, both works fine. :shipit:

# Features

- Structure based on directly calling reference functions within the controller object's events.
- Groups functions for runtime visualization and debugging.
- Allows for direct execution of the algorithm as desired.
- Enables step-by-step execution of the algorithm to visualize its operation incrementally.
- Algorithm optimized for multiple executions per frame (I recommend using YYC instead of the VM when comparing performance).

# Availabe Functions
-  `a_star(_instance, _target, _cell_size, _obstacles = [], _heuristic = manhattan)` Core of the execution.
-  `init()` You don't need to call this one; it simply initializes the structures that are used. 
-  `on_create()` Forces the correct initialization of the structures, used on Create Event:
-  `on_step()` Called on Step Event. Steps through the A* algorithm until the target is reached or no path is found.
-  ``

# Quick usage:

## Create Event:
- In the Create Event, call the script execution and assign it to a variable. `a = a_star(seeker_object, target_object, cell_size, [obstacle_array], heuristics);`
- Then call the `a.on_create()` function that trigger internalyy the `init()` setting the base data structures and parameters.
- If you wants the automatic execution just see the  `Step Event` topic.

# Step Event:
- Here you call `a.on_step()`, which will execute until it reaches the goal or exhausts the possible nodes for doing so.
- You can also use `a.on_activate();` in any event of your choice to execute the algorithm step-by-step, at a slow pace, to visualize it in action.

- Example: (Create Event)
  
```gml
move_speed  = 4;

a       = a_star(obj_enemy, obj_target, 32, [obj_wall], manhattan_tie_breaker);
a.on_create();
```

- Example: (Step Event)
```gml
a.on_step(); // Just runs the algorithm.
a.print_timers(); // Print execution timers.
a.move_instance(move_speed); // After A* execution, moves the obj_enemy to the obj_target with the specified move_speed.
```
