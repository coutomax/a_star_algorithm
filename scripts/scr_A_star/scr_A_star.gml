#macro DIR_X_ALL [-1, 1, 0, 0, -1, 1, -1, 1]
#macro DIR_Y_ALL [0, 0, -1, 1, -1, -1, 1, 1]

#macro DIR_X_MANHATTAN [-1, 1, 0, 0]
#macro DIR_Y_MANHATTAN [0, 0, -1, 1]

/// @desc                           Sets the properties of the grid used for A* pathfinding.
/// @param {Real} _cell_size        Size of each cell in the grid.
/// @param {Array} _obstacles       Array of obstacle instances.
/// @param {Array} _exceptions      Array of instances to be ignored as obstacles.
/// @returns {struct}               Returns a structure containing the grid properties and methods for A* pathfinding.
function set_grid_properties(_cell_size = 32, _obstacles = [], _exceptions = [])
{
    #region Variable declarations and callable structure
    g    =
    { 
        cell_size       : _cell_size,
        obstacles       : _obstacles,
        exceptions      : _exceptions,
        grid            : undefined,
        grid_width      : 0,
        grid_height     : 0,

        init           : function ()
        {
            var _status         = false;
            var _total_cells    = 0;

            if (cell_size <= 0)
            {
                show_debug_message("A* Error: Cell size must be greater than 0.");
                return _status;
            }

            if (array_length(obstacles) == 0)
            {
                show_debug_message("No obstacles provided. The grid will be created without any obstacles.");
            }

            grid_width      = room_width div cell_size;
            grid_height     = room_height div cell_size;

            var _total_cells    = grid_width * grid_height;

            // Initialize the data structures
            grid        = array_create(_total_cells, 0);

            return !_status;
        },

        // @desc Called when the grid needs to be created and filled with obstacles. Initializes the grid and fills it with the provided obstacles.
        create_grid     : function ()
        {
            if (!init()) return;
            
            _grid_filler(self);

            if (grid == undefined)
            {
                show_debug_message("A* Error: Grid is undefined.");
                return;
            }
        },

        // @desc Removes an instance from the grid, marking its occupied cells as empty.
        remove_instance     : function (_instance = noone)
        {
            if (_instance == noone) return;
            
            var _real_width  = sprite_get_width(_instance.sprite_index) * abs(_instance.image_xscale);
            var _real_height = sprite_get_height(_instance.sprite_index) * abs(_instance.image_yscale);

            if (_real_width == cell_size && _real_height == cell_size)
            {
                var _gx;
                var _x_adjust   = _instance.x;
                if (array_length(exceptions) > 0 && array_contains(exceptions, _instance.object_index))
                {
                    _x_adjust   = (_instance.image_xscale < 0) ? (_instance.x - cell_size) : _instance.x;
                }
                
                _gx             = _x_adjust div cell_size;
                var _gy         = _instance.y div cell_size;
                var _index      = _gx + (_gy * grid_width);

                grid[_index]    = 0;
            }
            else
            {   
                // If the sprite size is larger than the cell size, mark all cells covered by the sprite as empty
                var _width  = _real_width div cell_size;
                var _height = _real_height div cell_size;

                for (var _x = 0; _x < _width; _x++)
                {
                    for (var _y = 0; _y < _height; _y++)
                    {
                        var _gx;
                        var _x_adjust   = _instance.x;
                        if (array_length(exceptions) > 0 && array_contains(exceptions, _instance.object_index))
                        {
                            _x_adjust   = (_instance.image_xscale < 0) ? (_instance.x - cell_size) : _instance.x;
                        }
                        
                        _gx             = (_x_adjust + (_x * cell_size)) div cell_size;
                        var _gy         = (_instance.y + (_y * cell_size)) div cell_size;
                        var _index      = _gx + (_gy * grid_width);

                        grid[_index]    = 0;
                    }
                }    
            }       
        },
        
        // @desc Called on Draw Event. Draws the grid.
        draw_grid    : function (color = c_white)
        {
            draw_set_color(color);

            for (var _x = 0; _x < grid_width; _x++)
            {
                draw_line(_x * cell_size, 0, _x * cell_size, room_height);
                for (var _y = 0; _y < grid_height; _y++)
                {
                    draw_line(0, _y * cell_size, room_width, _y * cell_size);
                }
            }
        },

        // @desc Draws the coordinates of each cell in the grid.
        draw_coordinates : function (color = c_white)
        {
            draw_set_color(color);

            for (var _x = 0; _x < grid_width; _x++)
            {
                for (var _y = 0; _y < grid_height; _y++)
                {
                    var _gx = _x;
                    var _gy = _y;
                    var _x_pos = _gx * cell_size + cell_size / 2;
                    var _y_pos = _gy * cell_size + cell_size / 2;
                    draw_text_transformed(_x_pos - cell_size / 2, _y_pos - cell_size / 2, string(_gx) + "," + string(_gy), .6, .6, 1);
                }
            }
        },
    }
    #endregion

    return g;
}

/// @desc                           A* algorithm implementation for pathfinding in a grid-based environment.
/// @param {struct} _grid           The grid properties structure containing the grid and its dimensions.
/// @param {function} _heuristic    The heuristic function to be used for pathfinding (default is manhattan_tie_breaker).
/// @returns {struct}               Returns a structure containing methods and properties for executing the A* algorithm.
function a_star(_grid = undefined, _heuristic = manhattan_tie_breaker)
{
    #region Variable declarations and callable structure
    a   =
    {
        instance        : noone,
        target          : noone,
        heuristic       : _heuristic,
        grid_properties : _grid,
        grid            : undefined,
        open_set        : undefined,
        closed_set      : undefined,
        g_cost          : undefined,
        parent          : undefined,
        node            : undefined,
        path            : undefined,
        grid_width      : 0,
        grid_height     : 0,
        f_cost          : 0,
        visited_nodes   : 0,
        goal_gx         : 0,
        goal_gy         : 0,
        cell_size       : 0,
        reached         : false,
        failure         : false,
        timers          :
            {
                t0      : 0,
                t1      : 0,
                t2      : 0,
                t3      : 0,
                t4      : 0,
                t5      : 0,
                t6      : 0, 
                t7      : 0, 
                printed : false
            },

        // @desc Initialize the A* algorithm with the provided parameters.
        init        : function ()
        {
            timers.t0           = get_timer();
            var _status         = false;
            var _total_cells    = 0;

            if (heuristic == undefined)
            {
                show_debug_message("A* Error: Heuristic function is undefined.");
                return _status;
            }

            if (!is_callable(heuristic))
            {
                show_debug_message("A* Error: Heuristic function is not callable.");
                return _status;
            }

            if (grid_properties == undefined)
            {
                show_debug_message("A* Error: Grid is undefined.");
                return _status;
            }

            if (grid_properties.cell_size <= 0)
            {
                show_debug_message("A* Error: Cell size must be greater than 0.");
                return _status;
            }

            if (instance == noone)
            {
                show_debug_message("A* Error: Instance is undefined.");
                return _status;
            }

            if (target == noone)
            {
                show_debug_message("A* Error: Target is undefined.");
                return _status;
            }

            if (instance == target)
            {
                show_debug_message("A* Error: Instance and target are the same.");
                return _status;
            }

            cell_size       = grid_properties.cell_size;
            grid            = grid_properties.grid;
            grid_width      = grid_properties.grid_width;
            grid_height     = grid_properties.grid_height;

            var _total_cells    = grid_width * grid_height;
            var INF             = 1000000000; // A large value to represent infinity

            // Initialize the data structures
            open_set    = ds_priority_create();

            closed_set  = array_create(_total_cells, false);
            parent      = array_create(_total_cells, undefined);
            g_cost      = array_create(_total_cells, INF);

            var _gx     = instance.x  div cell_size;
            var _gy     = instance.y div cell_size;
            var _start_index    = _gx + (_gy * grid_width);

            ds_priority_add(open_set, _start_index, 0);

            goal_gx        = target.x div cell_size;
            goal_gy        = target.y div cell_size;

            closed_set[_start_index]    = true;
            g_cost[_start_index]        = 0;

            return !_status;
        },

        // @desc Adds an instance and its target to the A* algorithm for pathfinding.
        add_instances : function (_instance = noone, _target = noone)
        {
            instance    = _instance;
            target      = _target;

            return init();
        },

        // @desc Optional method to initialize the grid properties.
        grid_init    : function ()
        {
            if (grid_properties == undefined) 
            {
                show_debug_message("A* Error: Grid properties are undefined.");
                return;
            }
            
            cell_size       = grid_properties.cell_size;
            grid_width      = grid_properties.grid_width;
            grid_height     = grid_properties.grid_height;
        },

        // @desc Changes the grid properties used by the A* algorithm.
        change_grid : function (_grid = undefined)
        {
            if (_grid == undefined) return;

            grid_properties = _grid;
            grid_init();
            return init();
        },

        // @desc Called on Step Event. Steps through the A* algorithm until the target is reached or no path is found.
        path_finder     : function ()
        {
            if (instance == noone || target == noone) return;
            if (open_set == undefined || ds_priority_empty(open_set)) return;
            if (reached) return;

            _a_star_runner(self);            
        },
        
        // @desc Called when the A* algorithm is activated. Steps through the algorithm one step at a time.
        on_activate     : function ()
        {
            if (instance == noone || target == noone) return;
            if (open_set == undefined || ds_priority_empty(open_set)) return;
            if (reached) return;

            _step_by_step_runner(self);
        },

        // @desc Moves the instance along the calculated path towards the target at the specified speed.
        move_instance : function (speed = 1)
        {
            if (instance == noone || target == noone) return;
            if (path == undefined || ds_list_size(path) == 0) return;
            if (!reached) return;

            var next_node = path[| ds_list_size(path) - 1];
            var next_x = next_node.gx * cell_size + cell_size / 2;
            var next_y = next_node.gy * cell_size + cell_size / 2;

            var dx = next_x - instance.x;
            var dy = next_y - instance.y;

            if (abs(dx) < speed && abs(dy) < speed)
            {
                instance.x = next_x;
                instance.y = next_y;
                ds_list_delete(path, ds_list_size(path) - 1);
            }
            else
            {
                var angle   = point_direction(instance.x, instance.y, next_x, next_y);
                instance.x += lengthdir_x(speed, angle);
                instance.y += lengthdir_y(speed, angle);
            }

            if (ds_list_size(path) == 0)
            {
                clear(self);
            }
        },

        // @desc Prints the timing information for the A* algorithm's execution, including initialization, grid creation, A* execution, and path building times.
        print_timers : function ()
        {
            if (timers.printed) return;
            if (!reached && !failure) return;

            var _t_init = (timers.t1 - timers.t0) / 1000;
            var _t_create = (timers.t3 - timers.t2) / 1000;
            var _t_a_star = (timers.t5 - timers.t4) / 1000;
            var _t_path_build = (timers.t7 - timers.t6) / 1000;
            var _visited_nodes_count = visited_nodes > 0  ? string(visited_nodes) : " DISABLED (Step-by-step mode)"; //enabled only in step-by-step mode to prioritize performance

            show_debug_message("\n\n\nA* Timers:");
            show_debug_message("Initialization Time: " + string(_t_init) + " ms");
            show_debug_message("Grid Filling Time: " + string(_t_create) + " ms");
            show_debug_message("A* Execution Time: " + string(_t_a_star) + " ms");
            show_debug_message("Path Building Time: " + string(_t_path_build) + " ms");
            show_debug_message("Total Visited Nodes: " + _visited_nodes_count + "\n\n\n");

            timers.printed = true;
        },

        // @desc Clears the A* algorithm's data structures and resets the state.
        clear      : function ()
        {
            clear(self);
        },

        // @desc Draws the obstacles in the grid, highlighting the cells that are occupied by obstacles.
        draw_obstacles    : function (color = c_red)
        {
            if (grid == undefined) return;

            for (var _gx = 0; _gx < grid_width; _gx++) {
                for (var _gy = 0; _gy < grid_height; _gy++) {
                    var _idx = _gx + (_gy * grid_width);
                    
                    if (grid[_idx] != 0) {
                        draw_set_color(c_red);
                        draw_set_alpha(0.4);
                        draw_rectangle(_gx * 32, _gy * 32, (_gx + 1) * 32, (_gy + 1) * 32, false);
                        draw_set_alpha(1);
                    }
                }
            }
        },

        // @desc Draws the current node in the grid.
        draw_current_node : function (color = c_red)
        {
            if (node == undefined) return;

            var _gx     = node.gx;
            var _gy     = node.gy;
            var _last_gx   = node.last.gx;
            var _last_gy   = node.last.gy;
            var _x      = _gx * cell_size;
            var _y      = _gy * cell_size;

            var opacity = 0.4;
            draw_set_color(color);
            
            draw_set_alpha(opacity);
            draw_rectangle(_x, _y, _x + cell_size, _y + cell_size, false);
            draw_set_color(c_aqua);
            draw_rectangle(_last_gx * cell_size, _last_gy * cell_size, _last_gx * cell_size + cell_size, _last_gy * cell_size + cell_size, false);
            draw_set_alpha(1);
        },

        // @desc Draws the path from the target to the instance in the grid.
        draw_path : function (color = c_yellow)
        {
            if (path == undefined || ds_list_size(path) == 0) return;

            draw_set_color(color);

            var opacity = 0.4;
            draw_set_alpha(opacity);

            for (var i = 0; i < ds_list_size(path); i++)
            {
                var node = path[| i];
                var _gx = node.gx;
                var _gy = node.gy;
                var _x = _gx * cell_size;
                var _y = _gy * cell_size;

                draw_rectangle(_x, _y, _x + cell_size, _y + cell_size, false);
            }

            draw_set_alpha(1);
        },

        // @desc Draws the closed set in the grid.
        draw_closed_set : function (color = c_blue)
        {
            if (closed_set == undefined) return;
            if (array_length(closed_set) == 0) return;

            draw_set_color(color);
            var _opacity = 0.33;
            draw_set_alpha(_opacity);

            for (var i = 0; i < array_length(closed_set); i++)
            {
                if (closed_set[i])
                {
                    var _gx = i mod grid_width;
                    var _gy = i div grid_width;
                    var _x  = _gx * cell_size;
                    var _y  = _gy * cell_size;

                    draw_rectangle(_x, _y, _x + cell_size, _y + cell_size, false);
                }
            }
            
            draw_set_alpha(1);
        }
    }
    #endregion

    return a;
}

#region A* Algorithm Functions

/// @desc           Builds the path from the target to the instance using the parent grid.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
/// @returns {id}   Returns a ds_list containing the path from the target to the instance.
function _path_builder(_struct)
{
    _struct.timers.t6 = get_timer();

    var _path        = ds_list_create();
    var _grid_width  = _struct.grid_width;
    var _current_gx  = _struct.goal_gx;
    var _current_gy  = _struct.goal_gy;

    var _current_index  = _current_gx + (_current_gy * _grid_width);
    var _parent_array   = _struct.parent;

    while (_parent_array[_current_index] != undefined)
    {
        var node        = { gx: _current_gx, gy: _current_gy };
        ds_list_add(_path, node);

        _current_index  = _parent_array[_current_index];
        _current_gx     = _current_index mod _grid_width;
        _current_gy     = _current_index div _grid_width;
    }
    
    _struct.path        = _path;
    _struct.timers.t7   = get_timer();
}

/// @desc           Executes the A* algorithm step by step, for better visualization and debugging. Updates the open and closed sets.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
function _step_by_step_runner(_struct)
{
    var _instance       = _struct.instance;
    var _heuristic      = _struct.heuristic;

    var _dx_array       = _heuristic == manhattan || _heuristic == manhattan_tie_breaker ? DIR_X_MANHATTAN : DIR_X_ALL;
    var _dy_array       = _heuristic == manhattan || _heuristic == manhattan_tie_breaker ? DIR_Y_MANHATTAN : DIR_Y_ALL;
    var _dir_count      = array_length(_dx_array);

    var _grid_width     = _struct.grid_width;
    var _grid_height    = _struct.grid_height;

    var _goal_gx        = _struct.goal_gx;
    var _goal_gy        = _struct.goal_gy;

    var _grid           = _struct.grid;
    var _closed_set     = _struct.closed_set;
    var _open_set       = _struct.open_set;
    var _parent         = _struct.parent;
    var _g_cost         = _struct.g_cost;

    var _parent_node    = _struct.node;

    if(!ds_priority_empty(_open_set))
    {
        var _current    = ds_priority_delete_min(_open_set);
        var _current_gx = _current mod _grid_width;
        var _current_gy = _current div _grid_width;

        _closed_set[_current] = true;

        if (_current_gx == _goal_gx && _current_gy == _goal_gy)
        {
            _struct.reached     = true;
            _struct.node        = undefined;
            _path_builder(_struct);
            return;
        }

        var _current_g      = _g_cost[_current];

        for (var i = 0; i < _dir_count; i++)
        {
            var _n_gx       = _current_gx + _dx_array[i];
            var _n_gy       = _current_gy + _dy_array[i];

            if (_n_gx < 0 || _n_gx >= _grid_width || _n_gy < 0 || _n_gy >= _grid_height) continue;

            var _neighbor_id    = _n_gx + (_n_gy * _grid_width);

            if (_closed_set[_neighbor_id]) continue;
            if (_grid[_neighbor_id] != 0) continue;

            if (i >= 4)
            {
                var _adj_id1 = _n_gx + (_current_gy * _grid_width);
                var _adj_id2 = _current_gx + (_n_gy * _grid_width);

                if (_grid[_adj_id1] != 0 && _grid[_adj_id2] != 0) continue;
            }

            var _movement_cost  = i >= 4 ? 1.4 : 1; // Cost for diagonal movement (sqrt(2) * 1) or straight movement;
            var new_g           = _current_g + _movement_cost;

            _struct.visited_nodes++;

            _struct.node    = { gx: _n_gx, gy: _n_gy, last: _parent_node };
            _parent_node    = {gx: _current_gx, gy: _current_gy};

            if (new_g < _g_cost[_neighbor_id])
            {
                _g_cost[_neighbor_id] = new_g;
                _parent[_neighbor_id] = _current;

                var hn          = _heuristic(_n_gx, _n_gy, _goal_gx, _goal_gy);
                var f_cost      = new_g + hn;

                ds_priority_add(_open_set, _neighbor_id, f_cost);
            }
        }
    }

    if (ds_priority_empty(_open_set) && !_struct.reached && !_struct.failure)
    {
        _struct.failure     = true;
        _struct.node        = undefined;
        return;
    }
}

/// @desc           Executes the A* algorithm until completion, updating the open and closed sets.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
function _a_star_runner(_struct)
{
    _struct.timers.t4   = get_timer();

    var _instance       = _struct.instance;
    var _heuristic      = _struct.heuristic;

    var _dx_array       = _heuristic == manhattan || _heuristic == manhattan_tie_breaker ? DIR_X_MANHATTAN : DIR_X_ALL;
    var _dy_array       = _heuristic == manhattan || _heuristic == manhattan_tie_breaker ? DIR_Y_MANHATTAN : DIR_Y_ALL;
    var _dir_count      = array_length(_dx_array);

    var _grid_width     = _struct.grid_width;
    var _grid_height    = _struct.grid_height;

    var _goal_gx        = _struct.goal_gx;
    var _goal_gy        = _struct.goal_gy;

    var _grid           = _struct.grid;
    var _closed_set     = _struct.closed_set;
    var _open_set       = _struct.open_set;
    var _parent         = _struct.parent;
    var _g_cost         = _struct.g_cost;

    while(!ds_priority_empty(_open_set))
    {
        var _current    = ds_priority_delete_min(_open_set);
        var _current_gx = _current mod _grid_width;
        var _current_gy = _current div _grid_width;

        _closed_set[_current] = true;

        if (_current_gx == _goal_gx && _current_gy == _goal_gy)
        {
            _struct.reached = true;
            _struct.timers.t5       = get_timer();
            _path_builder(_struct);
            return;
        }

        var _current_g      = _g_cost[_current];

        for (var i = 0; i < _dir_count; i++)
        {
            var _n_gx       = _current_gx + _dx_array[i];
            var _n_gy       = _current_gy + _dy_array[i];

            if (_n_gx < 0 || _n_gx >= _grid_width || _n_gy < 0 || _n_gy >= _grid_height) continue;

            var _neighbor_id    = _n_gx + (_n_gy * _grid_width);

            if (_closed_set[_neighbor_id]) continue;
            if (_grid[_neighbor_id] != 0) continue;

            if (i >= 4)
            {
                var _adj_id1 = _n_gx + (_current_gy * _grid_width);
                var _adj_id2 = _current_gx + (_n_gy * _grid_width);

                if (_grid[_adj_id1] != 0 && _grid[_adj_id2] != 0) continue;
            }

            var _movement_cost  = i >= 4 ? 1.4 : 1; // Cost for diagonal movement (sqrt(2) * 1) or straight movement;
            var new_g           = _current_g + _movement_cost;

            if (new_g < _g_cost[_neighbor_id])
            {
                _g_cost[_neighbor_id]   = new_g;
                _parent[_neighbor_id]   = _current;

                var hn          = _heuristic(_n_gx, _n_gy, _goal_gx, _goal_gy);
                var f_cost      = new_g + hn;

                ds_priority_add(_open_set, _neighbor_id, f_cost);
            }
        }
    }

    if (ds_priority_empty(_open_set) && !_struct.reached && !_struct.failure)
    {
        _struct.failure = true;
        _struct.timers.t5       = get_timer();
        return;
    }
}

#endregion

#region Helper Functions

/// @desc               Clears the A* algorithm's data structures and resets the state.
/// @param {any*}       _struct The structure containing data for the A* algorithm.
function clear(_struct)
{
    if (_struct.open_set == undefined) return;
    if (_struct.closed_set == undefined) return;

    ds_priority_destroy(_struct.open_set);

    with(_struct)
    {
        var _total_cells    = grid_width * grid_height;
        var INF             = 1000000000;

        open_set    = ds_priority_create();

        closed_set  = array_create(_total_cells, false);
        parent      = array_create(_total_cells, undefined);
        g_cost      = array_create(_total_cells, INF);

        var _gx     = instance.x  div cell_size;
        var _gy     = instance.y div cell_size;
        var _start_index    = _gx + (_gy * grid_width);

        ds_priority_add(open_set, _start_index, 0);

        goal_gx        = target.x div cell_size;
        goal_gy        = target.y div cell_size;

        closed_set[_start_index]    = true;
        g_cost[_start_index]        = 0;

        node            = undefined;
        path            = undefined;
        f_cost          = 0;
        reached         = false;
        failure         = false;
        visited_nodes   = 0;
    }
}

/// @desc                           Fills the grid with the given instance, target, and obstacles and updates the grid structure accordingly.
/// @param {any*}                   _struct The structure containing data for the A* algorithm.
function _grid_filler(_struct)
{
    var _obstacles_count    = array_length(_struct.obstacles);
    var exceptions          = _struct.exceptions;
    var _cell_size          = _struct.cell_size;
    var _grid               = _struct.grid;
    var _grid_width         = _struct.grid_width;
    var _grid_height        = _struct.grid_height;

    if (_obstacles_count <= 0) return;
    
    for (var i = 0; i < _obstacles_count; i++)
    {
        var obstacle = _struct.obstacles[i];

        if (!instance_exists(obstacle)) continue;
        with (obstacle)
        {   
            var _real_width  = sprite_get_width(sprite_index) * abs(image_xscale);
            var _real_height = sprite_get_height(sprite_index) * abs(image_yscale);
            // Check if the sprite size matches the cell size
            if (_real_width == _cell_size && _real_height == _cell_size)
            {

                var _gx;
                var _x_adjust   = x;
                if (array_length(exceptions) > 0 && array_contains(exceptions, object_index))
                {
                    _x_adjust   = (image_xscale < 0) ? (x - _cell_size) : x;
                }
                
                _gx             = _x_adjust div _cell_size;
                var _gy         = y div _cell_size;
                var _index      = _gx + (_gy * _grid_width);

                _grid[_index]    = object_index;
            }
            else
            {   
                // If the sprite size is larger than the cell size, mark all cells covered by the sprite as obstacles
                var _width  = _real_width div _cell_size;
                var _height = _real_height div _cell_size;

                for (var _x = 0; _x < _width; _x++)
                {
                    for (var _y = 0; _y < _height; _y++)
                    {
                        var _gx;
                        var _x_adjust   = x;
                        if (array_length(exceptions) > 0 && array_contains(exceptions, object_index))
                        {
                            _x_adjust   = (image_xscale < 0) ? (x - _real_width - 1) : x;
                        }
                
                        _gx             = (_x_adjust + (_x * _cell_size)) div _cell_size;
                        var _gy         = (y + (_y * _cell_size)) div _cell_size;
                        var _index      = _gx + (_gy * _grid_width);

                        _grid[_index]    = object_index;
                    }
                }
            }
        }
    }

    _struct.grid    = _grid;
}

#endregion

#region Heuristic Functions

/// @desc               Returns the heuristic based on Manhattan distance method.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function manhattan (x1, y1, x2, y2) // used when able to move in 4 directions
{
    gml_pragma("forceinline");
    return abs(x2 - x1) + abs(y2 - y1);
}

/// @desc               Returns the heuristic based on Manhattan distance method, but with a slight bias to prefer straight paths.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function manhattan_tie_breaker (x1, y1, x2, y2) // used when able to move in 4 directions
{
    gml_pragma("forceinline");
    var _dist = abs(x2 - x1) + abs(y2 - y1);
    return _dist * 1.001;
}

/// @desc               Returns the heuristic based on Euclidean distance method.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function euclidean (x1, y1, x2, y2) // used when able to move in 8 directions
{
    gml_pragma("forceinline");
    return sqrt(sqr(x2 - x1) + sqr(y2 - y1));
}

/// @desc               Returns the heuristic based on Octile distance method.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function octile (x1, y1, x2, y2) // used when able to move in 8 directions
{
    var _h_dx = abs(x2 - x1);
    var _h_dy = abs(y2 - y1);
    var _min_val = (_h_dx < _h_dy) ? _h_dx : _h_dy;
    return ((_h_dx + _h_dy) + (-0.5857864) * _min_val) * 1.001;
}

/// @desc               Returns the heuristic based on Chebyshev distance method.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function chebyshev (x1, y1, x2, y2) // used when able to move in 8 directions
{
    gml_pragma("forceinline");
    var dx = abs(x2 - x1);
    var dy = abs(y2 - y1);
    return max(dx, dy);
}

#endregion