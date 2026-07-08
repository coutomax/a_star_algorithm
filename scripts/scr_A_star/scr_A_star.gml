
#macro DIR_X_ALL [-1, 1, 0, 0, -1, 1, -1, 1]
#macro DIR_Y_ALL [0, 0, -1, 1, -1, -1, 1, 1]

#macro DIR_X_MANHATTAN [-1, 1, 0, 0]
#macro DIR_Y_MANHATTAN [0, 0, -1, 1]

/// @desc                   A* algorithm implementation for pathfinding in a grid-based environment.
/// @param {Instance}       _instance Current instance that will be move towards the target.
/// @param {Instance}       _target Target instance that the current instance will move towards.
/// @param {Real}           _cell_size Size of each cell in the grid.
/// @param {Array}          _obstacles Array of obstacle instances.
/// @param {Boolean}        _active_debug Whether to enable debug messages.
function a_star(_instance, _target, _cell_size, _obstacles = [], _heuristic = manhattan)
{
    #region Variable declarations and callable structure
    a   =
    { 
        instance        : _instance,
        target          : _target,
        cell_size       : _cell_size,
        obstacles       : _obstacles,
        heuristic       : _heuristic,
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
        operations      : 0,
        goal_gx         : 0,
        goal_gy         : 0,
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

            if (instance == noone || target == noone) 
            {
                show_debug_message("A* Error: Instance or target is noone.");
                return _status;
            }

            if (cell_size <= 0)
            {
                show_debug_message("A* Error: Cell size must be greater than 0.");
                return _status;
            }

            if (instance == target)
            {
                show_debug_message("A* Error: Instance and target are the same.");
                return _status;
            }

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

            if (array_length(obstacles) == 0)
            {
                show_debug_message("No obstacles provided. The grid will be created without any obstacles.");
            }      

            grid_width      = room_width div cell_size;
            grid_height     = room_height div cell_size;

            var _total_cells    = grid_width * grid_height;
            var INF             = 1000000000; // A large value to represent infinity

            // Initialize the data structures
            open_set    = ds_priority_create();
            grid        = array_create(_total_cells, 0);
            closed_set  = array_create(_total_cells, false);
            parent      = array_create(_total_cells, undefined);
            g_cost      = array_create(_total_cells, INF);

            var actual_location = _actual_location(self);
            var _start_gx       = real(actual_location.gx);
            var _start_gy       = real(actual_location.gy);
            var _start_index    = _start_gx + (_start_gy * grid_width);

            ds_priority_add(open_set, _start_index, 0);

            goal_gx        = target.x div cell_size;
            goal_gy        = target.y div cell_size;

            closed_set[_start_index]    = true;
            g_cost[_start_index]        = 0;

            return !_status;
        },
        
        // @desc Called when the A* algorithm is created. Initializes the grid and fills it with obstacles.
        on_create    : function ()
        {
            if (!init()) return;

            timers.t1   = get_timer();
            
            var actual_location = _actual_location(self);

            _grid_filler(self);
        },

        // @desc Called on Step Event. Steps through the A* algorithm until the target is reached or no path is found.
        on_step     : function ()
        {
            if (instance == noone || target == noone) return;
            if (open_set == undefined || ds_priority_empty(open_set)) return;
            if (reached) return;

            a_star_runner(self);            
        },
        
        // @desc Called when the A* algorithm is activated. Steps through the algorithm one step at a time.
        on_activate     : function ()
        {
            if (instance == noone || target == noone) return;
            if (open_set == undefined || ds_priority_empty(open_set)) return;
            if (reached) return;

            _step_by_step_runner(self);
        },

        move_instance : function (speed = 1)
        {
            if (instance == noone || target == noone) return;
            if (path == undefined || ds_list_size(path) == 0) return;

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
                var angle = point_direction(instance.x, instance.y, next_x, next_y);
                instance.x += lengthdir_x(speed, angle);
                instance.y += lengthdir_y(speed, angle);
            }

            if (ds_list_size(path) == 0)
            {
                clear(self);
            }
        },

        print_timers : function ()
        {
            if (timers.printed) return;

            var _t_init = (timers.t1 - timers.t0) / 1000;
            var _t_create = (timers.t3 - timers.t2) / 1000;
            var _t_a_star = (timers.t5 - timers.t4) / 1000;
            var _t_path_build = (timers.t7 - timers.t6) / 1000;

            show_debug_message("A* Timers:");
            show_debug_message("Initialization Time: " + string(_t_init) + " ms");
            show_debug_message("Grid Filling Time: " + string(_t_create) + " ms");
            show_debug_message("A* Execution Time: " + string(_t_a_star) + " ms");
            show_debug_message("Path Building Time: " + string(_t_path_build) + " ms");
            show_debug_message("Total Operations: " + string(operations));

            timers.printed = true;
        },

        // @desc Clears the A* algorithm's data structures and resets the state.
        clear      : function ()
        {
            clear(self);
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

        // @desc Draws the current node in the grid.
        draw_current_node : function (color = c_red)
        {
            if (node == undefined) return;

            var _gx     = node.gx;
            var _gy     = node.gy;
            var _x      = _gx * cell_size;
            var _y      = _gy * cell_size;

            var opacity = 0.4;
            draw_set_color(color);
            
            draw_set_alpha(opacity);
            draw_rectangle(_x, _y, _x + cell_size, _y + cell_size, false);
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

    var current_index = _current_gx + (_current_gy * _grid_width);
    var parent_array   = _struct.parent;

    while (parent_array[current_index] != undefined)
    {
        var node = { gx: _current_gx, gy: _current_gy };
        ds_list_add(_path, node);

        current_index = parent_array[current_index];
        _current_gx   = current_index mod _grid_width;
        _current_gy   = current_index div _grid_width;
    }
    
    _struct.path        = _path;
    _struct.timers.t7   = get_timer();
}

/// @desc           Executes the A* algorithm step by step, updating the open and closed sets.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
function _step_by_step_runner(_struct)
{
    var _instance       = _struct.instance;
    var _heuristic      = _struct.heuristic;

    var _actual_gx = _instance.x div _struct.cell_size;
    var _actual_gy = _instance.y div _struct.cell_size;

    var _dx_array       = _heuristic == manhattan ? DIR_X_MANHATTAN : DIR_X_ALL;
    var _dy_array       = _heuristic == manhattan ? DIR_Y_MANHATTAN : DIR_Y_ALL;
    var _dir_count      = array_length(_dx_array);

    var _grid_width      = _struct.grid_width;
    var _grid_height     = _struct.grid_height;

    var _goal_gx        = _struct.goal_gx;
    var _goal_gy        = _struct.goal_gy;

    var _grid           = _struct.grid;
    var _closed_set     = _struct.closed_set;
    var _open_set       = _struct.open_set;
    var _parent         = _struct.parent;
    var _g_cost         = _struct.g_cost;
    
    var _reached        = _struct.reached;
    var _failure        = _struct.failure;
    var _reached_nodes  = 0;

    if(!ds_priority_empty(_open_set))
    {
        var _current = ds_priority_delete_min(_open_set);
        var _current_gx = _current mod _grid_width;
        var _current_gy = _current div _grid_width;

        _closed_set[_current] = true;
        _reached_nodes++;

        if (_current_gx == _goal_gx && _current_gy == _goal_gy)
        {
            _struct.reached = true;
            _struct.operations = _reached_nodes;
            _path_builder(_struct);
            return;
        }

        var _current_g = _g_cost[_current];

        for (var i = 0; i < _dir_count; i++)
        {
            var _n_gx = _current_gx  + _dx_array[i];
            var _n_gy = _current_gy + _dy_array[i];

            if (_n_gx < 0 || _n_gx >= _grid_width || _n_gy < 0 || _n_gy >= _grid_height) continue;

            var _neighbor_id    = _n_gx + (_n_gy * _grid_width);

            if (_struct.closed_set[_neighbor_id]) continue;
            if (_struct.grid[_neighbor_id] != 0) continue;

            var _movement_cost  = i > 4 ? 1.4 : 1; // Cost for diagonal movement (sqrt(2) * 1) or straight movement;
            var new_g           = _current_g + _movement_cost;

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
        _struct.failure = true;
        _struct.operations = _reached_nodes;
        return;
    }
}

/// @desc           Executes the A* algorithm until completion, updating the open and closed sets.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
function a_star_runner(_struct)
{
    _struct.timers.t4   = get_timer();
    var _instance       = _struct.instance;
    var _heuristic      = _struct.heuristic;

    var _actual_gx = _instance.x div _struct.cell_size;
    var _actual_gy = _instance.y div _struct.cell_size;

    var _dx_array       = _heuristic == manhattan ? DIR_X_MANHATTAN : DIR_X_ALL;
    var _dy_array       = _heuristic == manhattan ? DIR_Y_MANHATTAN : DIR_Y_ALL;
    var _dir_count      = array_length(_dx_array);

    var _grid_width      = _struct.grid_width;
    var _grid_height     = _struct.grid_height;

    var _goal_gx        = _struct.goal_gx;
    var _goal_gy        = _struct.goal_gy;

    var _grid           = _struct.grid;
    var _closed_set     = _struct.closed_set;
    var _open_set       = _struct.open_set;
    var _parent         = _struct.parent;
    var _g_cost         = _struct.g_cost;
    
    var _reached        = _struct.reached;
    var _failure        = _struct.failure;
    var _reached_nodes  = 0;

    while(!ds_priority_empty(_open_set))
    {
        var _current = ds_priority_delete_min(_open_set);
        var _current_gx = _current mod _grid_width;
        var _current_gy = _current div _grid_width;

        _closed_set[_current] = true;
        _reached_nodes++;

        if (_current_gx == _goal_gx && _current_gy == _goal_gy)
        {
            _struct.reached = true;
            timers.t5       = get_timer();
            _struct.operations = _reached_nodes;
            _path_builder(_struct);
            return;
        }

        var _current_g = _g_cost[_current];

        for (var i = 0; i < _dir_count; i++)
        {
            var _n_gx = _current_gx  + _dx_array[i];
            var _n_gy = _current_gy + _dy_array[i];

            if (_n_gx < 0 || _n_gx >= _grid_width || _n_gy < 0 || _n_gy >= _grid_height) continue;

            var _neighbor_id    = _n_gx + (_n_gy * _grid_width);

            if (_struct.closed_set[_neighbor_id]) continue;
            if (_struct.grid[_neighbor_id] != 0) continue;

            var _movement_cost  = i > 4 ? 1.4 : 1; // Cost for diagonal movement (sqrt(2) * 1) or straight movement;
            var new_g           = _current_g + _movement_cost;

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
        _struct.failure = true;
        timers.t5       = get_timer();
        _struct.operations = _reached_nodes;
        return;
    }
}

#endregion

#region Helper Functions

/// @desc               Clears the A* algorithm's data structures and resets the state.
/// @param {any*}       _struct The structure containing data for the A* algorithm.
function clear(_struct)
{
    ds_priority_destroy(_struct.open_set);

    _struct.grid        = undefined;
    _struct.open_set    = undefined;
    _struct.closed_set  = undefined;
    _struct.g_cost      = undefined;
    _struct.parent      = undefined;
    _struct.node        = undefined;
    _struct.path        = undefined;
    _struct.f_cost      = 0;
    _struct.reached     = false;
    _struct.failure     = false;
    _struct.operations  = 0;
}

/// @desc                           Fills the grid with the given instance, target, and obstacles.
/// @param {Instance} instance      The instance to place in the grid.
/// @param {Instance} target        The target instance to place in the grid.
/// @param {Array} [obstacles]=[]   An array of obstacle instances to place in the grid.
/// @param {Real} cell_size         The size of each cell in the grid.
function _grid_filler(_struct)
{
    _struct.timers.t2           = get_timer();
    var _obstacles_count    = array_length(_struct.obstacles);
    var _cell_size          = _struct.cell_size;
    var _grid               = _struct.grid;
    var _grid_width         = _struct.grid_width;
    var _grid_height        = _struct.grid_height;

    if (_obstacles_count <= 0) return;
    
    for (var i = 0; i < _obstacles_count; i++)
    {
        var instance = _struct.obstacles[i];

        if (instance == noone) continue;        
        with (instance)
        {   
            // Check if the sprite size matches the cell size
            if (sprite_width == _cell_size && sprite_height == _cell_size)
            {
                var _gx     = x div _cell_size;
                var _gy     = y div _cell_size;
                var _index  = _gx + (_gy * _grid_width);

                _grid[_index]    = object_index;
            }
            else
            {   
                // If the sprite size is larger than the cell size, mark all cells covered by the sprite as obstacles
                var _width  = sprite_width div _cell_size;
                var _height = sprite_height div _cell_size;

                for (var _x = 0; _x < _width; _x++)
                {
                    for (var _y = 0; _y < _height; _y++)
                    {
                        var _gx     = (x + (_x * _cell_size)) div _cell_size;
                        var _gy     = (y + (_y * _cell_size)) div _cell_size;
                        var _index  = _gx + (_gy * _grid_width);

                        _grid[_index]    = object_index;
                    }
                }
            }
        }
    }
    _struct.timers.t3           = get_timer();
}

/// @desc                   Returns the actual grid coordinates of the given instance based on its position and cell size.
/// @param {any*}           _struct The structure containing data for the A* algorithm, including the instance and cell size.
/// @returns {struct}        Returns a structure containing the grid coordinates of the instance.
function _actual_location(_struct)
{
    gml_pragma("forceinline");
    var _gx     = _struct.instance.x div _struct.cell_size;
    var _gy     = _struct.instance.y div _struct.cell_size;

    return {
        gx: string(_gx),
        gy: string(_gy)
    };
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


/// @desc               Returns the heuristic based on Euclidean distance method.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function euclidean (x1, y1, x2, y2) // used when able to move in 8 directions and has no obstacles
{
    gml_pragma("forceinline");
    return sqrt(sqr(x2 - x1) + sqr(y2 - y1));
}

/// @desc               Returns the heuristic based on point_distance inbuilt method.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function simple_euclidian (x1, y1, x2, y2) // used when able to move in 8 directions and has no obstacles
{
    gml_pragma("forceinline");
    return point_distance(x1, y1, x2, y2);
}

/// @desc               Returns the heuristic based on Octile distance method.
/// @param {Real} x1    X of starting position.
/// @param {Real} y1    Y of starting position.
/// @param {Real} x2    X of target position.
/// @param {Real} y2    Y of target position.
/// @returns {Real}     Returns the value that references the cost to reach the goal.
function octile (x1, y1, x2, y2) // used when able to move in 8 directions
{
    gml_pragma("forceinline");
    var dx = abs(x2 - x1);
    var dy = abs(y2 - y1);
    return (dx + dy) + (sqrt(2) - 2) * min(dx, dy);
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