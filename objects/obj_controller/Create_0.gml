toggle      = false;
move_speed  = 4;

global.grid_properties = set_grid_properties(32, [obj_wall]);
global.grid_properties.create_grid(); // Used to check when the global.grid changes, and to create the grid when the game starts.
global.grid = variable_clone(global.grid_properties);