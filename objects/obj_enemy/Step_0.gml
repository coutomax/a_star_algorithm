a.path_finder();
a.move_instance(move_speed);

if (!array_equals(original_grid.grid, global.grid.grid))
{
    a.clear();
    original_grid = variable_clone(global.grid);
    a.change_grid(original_grid);
}