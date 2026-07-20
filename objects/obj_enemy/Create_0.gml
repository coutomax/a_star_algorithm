depth       = 1;
move_speed  = 4;

original_grid = variable_clone(global.grid);

a = a_star(original_grid);

a.add_instances(id, obj_target);