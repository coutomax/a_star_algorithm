/// @desc                   A* algorithm implementation for pathfinding in a grid-based environment.
/// @param {Instance}       _instance Current instance that will be move towards the target.
/// @param {Instance}       _target Target instance that the current instance will move towards.
/// @param {Real}           _cell_size Size of each cell in the grid.
/// @param {Array}          _obstacles Array of obstacle instances.
/// @param {Boolean}        _active_debug Whether to enable debug messages.
function a_star(_instance, _target, _cell_size, _obstacles = [], _heuristic = manhattan, _active_debug = false)
{
    #region Variables and callable functions
    a   =
    {
        instance        : _instance,
        target          : _target,
        cell_size       : _cell_size,
        obstacles       : _obstacles,
        heuristic       : _heuristic,
        active_debug    : _active_debug,
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
        reached         : false,
        failure         : false,
        operations      : 0,

        // @desc Initialize the A* algorithm with the provided parameters.
        init        : function ()
        {
            gml_pragma("forceinline");
            var _status = false;

            if (instance == noone || target == noone) 
            {
                debug_message("A* Error: Instance or target is noone.");
                return _status;
            }

            if (cell_size <= 0)
            {
                debug_message("A* Error: Cell size must be greater than 0.");
                return _status;
            }

            if (instance == target)
            {
                debug_message("A* Error: Instance and target are the same.");
                return _status;
            }

            if (heuristic == undefined)
            {
                debug_message("A* Error: Heuristic function is undefined.");
                return _status;
            }

            if (!is_callable(heuristic))
            {
                debug_message("A* Error: Heuristic function is not callable.");
                return _status;
            }

            if (array_length(obstacles) == 0)
            {
                debug_message("No obstacles provided. The grid will be created without any obstacles.");
            }          

            grid_width      = room_width div cell_size;
            grid_height     = room_height div cell_size;
            open_set        = ds_priority_create();

            // Create the grid with the specified width and height
            grid            = ds_grid_create(grid_width, grid_height);
            closed_set      = ds_grid_create(grid_width, grid_height); 
            g_cost          = ds_grid_create(grid_width, grid_height);
            parent          = ds_grid_create(grid_width, grid_height);

            ds_grid_clear(closed_set, false); // Initialize closed_set with false values
            ds_grid_clear(parent, undefined); // Initialize parent with undefined values

            var INF = 1000000000; // A large value to represent infinity
            ds_grid_clear(g_cost, INF);

            var actual_location = _actual_location(self);
            closed_set[# real(actual_location.gx), real(actual_location.gy)]    = true;
            g_cost[# real(actual_location.gx), real(actual_location.gy)]        = 0;

            return !_status;
        },
        
        // @desc Called when the A* algorithm is created. Initializes the grid and fills it with obstacles.
        on_create    : function ()
        {
            if (!init()) return;
            
            var actual_location = _actual_location(self);

            _grid_filler(self);
            _add_open_set(self, actual_location.gx, actual_location.gy);
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
                debug_message("A* Info: Instance has reached the target.");
            }
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
                    var _px = _x * cell_size;
                    var _py = _y * cell_size;
                    var text = "(" + string(_x) + "," + string(_y) + ")";
                    draw_text_transformed(_px + 4, _py + 4, text, 0.5, 0.5, 1);
                }
            }
        },

        // @desc Draws the closed set in the grid.
        draw_closed_set : function (color = c_blue)
        {
            if (closed_set == undefined) return;

            draw_set_color(color);
            var _opacity = 0.33;
            draw_set_alpha(_opacity);

            for (var _x = 0; _x < grid_width; _x++)
            {
                for (var _y = 0; _y < grid_height; _y++)
                {
                    if (closed_set[# _x, _y])
                    {
                        var _px = _x * cell_size;
                        var _py = _y * cell_size;
                        draw_rectangle(_px, _py, _px + cell_size, _py + cell_size, false);
                    }
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
    var path = ds_list_create();
    var current_gx = _struct.target.x div _struct.cell_size;
    var current_gy = _struct.target.y div _struct.cell_size;

    while(_struct.parent[# current_gx, current_gy] != undefined)
    {
        ds_list_add(path, { gx: current_gx, gy: current_gy });
        var parent_index = _struct.parent[# current_gx, current_gy];
        current_gx = parent_index mod _struct.grid_width;
        current_gy = parent_index div _struct.grid_width;
    }
    return path;
}

/// @desc           Executes the A* algorithm step by step, updating the open and closed sets.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
function _step_by_step_runner(_struct)
{
    if (ds_priority_empty(_struct.open_set)) return;

    var current = ds_priority_delete_min(_struct.open_set);
    var current_gx = current mod _struct.grid_width;
    var current_gy = current div _struct.grid_width;

    debug_message("Current node: (" + string(current_gx) + ", " + string(current_gy) + ")");

    _struct.node = { gx: current_gx, gy: current_gy };
    _struct.closed_set[# current_gx, current_gy] = true;

    debug_message($"Closed set updated: Node (" + string(current_gx) + ", " + string(current_gy) + ") added to closed set.");

    var goal_gx = _struct.target.x div _struct.cell_size;
    var goal_gy = _struct.target.y div _struct.cell_size;

    if (current_gx == goal_gx && current_gy == goal_gy)
    {
        _struct.reached = true;
        _struct.path = _path_builder(_struct);
        debug_message("\nA* Success: Path found to target at (" + string(goal_gx) + ", " + string(goal_gy) + ")\n");
        return;
    }

    _add_open_set(_struct, current_gx, current_gy);
}

/// @desc           Executes the A* algorithm until completion, updating the open and closed sets.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
function a_star_runner(_struct)
{
    var goal_gx = _struct.target.x div _struct.cell_size;
    var goal_gy = _struct.target.y div _struct.cell_size;

    while (!ds_priority_empty(_struct.open_set))
    {
        var current = ds_priority_delete_min(_struct.open_set);
        var current_gx = current mod _struct.grid_width;
        var current_gy = current div _struct.grid_width;

        _struct.node = { gx: current_gx, gy: current_gy };
        _struct.closed_set[# current_gx, current_gy] = true;

        if (current_gx == goal_gx && current_gy == goal_gy)
        {
            _struct.reached = true;
            _struct.path = _path_builder(_struct);
            debug_message("\nA* Success: Path found to target at (" + string(goal_gx) + ", " + string(goal_gy) + ")\n");
            return;
        }

        _add_open_set(_struct, current_gx, current_gy);
    }

    if (ds_priority_empty(_struct.open_set) && !_struct.reached && !_struct.failure)
    {
        _struct.failure = true;
        debug_message("\nA* Failure: No path found to target at (" + string(goal_gx) + ", " + string(goal_gy) + ")\n");
    }
}

/// @desc           Adds neighboring nodes to the open set for the A* algorithm.
/// @param {any*}   _struct The structure containing data for the A* algorithm.
/// @param {Real} [_gx]=-1 The x-coordinate of the current node.
/// @param {Real} [_gy]=-1 The y-coordinate of the current node.
function _add_open_set (_struct, _gx = -1, _gy = -1)
{
    var _directions;
    var _movement_cost;
    var _actual_gx = real(_gx);
    var _actual_gy = real(_gy);

    if (_gx == -1 || _gy == -1)
    {
        var actual_location = _actual_location(_struct);
        _actual_gx = real(actual_location.gx);
        _actual_gy = real(actual_location.gy);
    }

    if (_struct.heuristic == manhattan)
    {
        _directions     = [
            { dx: -1, dy: 0 }, // left
            { dx: 1, dy: 0 },  // right
            { dx: 0, dy: -1 }, // up
            { dx: 0, dy: 1 }   // down
        ];
    }
    else
    {
        _directions     = [
            { dx: -1, dy: 0 }, // left
            { dx: 1, dy: 0 },  // right
            { dx: 0, dy: -1 }, // up
            { dx: 0, dy: 1 },  // down
            { dx: -1, dy: -1 }, // up-left
            { dx: 1, dy: -1 },  // up-right
            { dx: -1, dy: 1 },  // down-left
            { dx: 1, dy: 1 }    // down-right
        ];
    }

    for (var i = 0; i < array_length(_directions); i++)
    {
        var _dir = _directions[i];
        var _n_gx = _actual_gx + _dir.dx;
        var _n_gy = _actual_gy + _dir.dy;

        if (!_is_room_cell(_n_gx, _n_gy, _struct.grid)) 
            continue;
        if (_struct.closed_set[# _n_gx, _n_gy]) 
            continue;
        if (!_free_path(_struct, _n_gx, _n_gy))
            continue;
               
        _movement_cost = (abs(_dir.dx) + abs(_dir.dy) == 2) ? 14 : 10; // Cost for diagonal movement (sqrt(2) * 10) or straight movement;

        var hn          = _struct.heuristic(_n_gx, _n_gy, _struct.target.x div _struct.cell_size, _struct.target.y div _struct.cell_size);
        var current_g   = _struct.g_cost[# _actual_gx, _actual_gy];
        var new_g       = current_g + _movement_cost;

        if (new_g < _struct.g_cost[# _n_gx, _n_gy])
        {
            _struct.g_cost[# _n_gx, _n_gy] = new_g;
            _struct.parent[# _n_gx, _n_gy] = _actual_gx + (_actual_gy * _struct.grid_width);

            var f_cost = new_g + hn;

            ds_priority_add(_struct.open_set, _n_gx + (_n_gy * _struct.grid_width), f_cost);
        }
    }
}

#endregion

#region Helper Functions

/// @desc           Displays a debug message if active_debug is true.
/// @param {String} _message The message to display in the debug console.
function debug_message(_message)
{
    if (active_debug)
    {
        show_debug_message(_message);
    }
}

/// @desc               Clears the A* algorithm's data structures and resets the state.
/// @param {any*}       _struct The structure containing data for the A* algorithm.
function clear(_struct)
{
    ds_grid_destroy(_struct.grid);
    ds_grid_destroy(_struct.closed_set);
    ds_grid_destroy(_struct.g_cost);
    ds_grid_destroy(_struct.parent);
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
    if (array_length(_struct.obstacles) > 0)
    {
        for (var i = 0; i < array_length(_struct.obstacles); i++)
        {
            var _obs = _struct.obstacles[i];
            if (instance_exists(_obs))
            {
                _object_grid_location(_obs, _struct.cell_size, _struct.grid);
            }
            else
            {
                debug_message("A* Error: Obstacle instance does not exist.");
                return;
            }
        }
    }
}

/// @desc                               Sets the grid coordinates of the given instance.
/// @param {Instance} instance          The instance to set the grid coordinates for.
/// @param {Real} cell_size             The size of each cell in the grid.
/// @param {id.dsgrid<any*>} grid       The grid to store the instance's location in.
function _object_grid_location(instance, cell_size, grid)
{
    with (instance)
    {   
        // Check if the sprite size matches the cell size
        if (sprite_width == other.cell_size && sprite_height == other.cell_size)
        {
            var _gx     = x div other.cell_size;
            var _gy     = y div other.cell_size;

            other.grid[# _gx, _gy]    = object_index;
        }
        else
        {   
            // If the sprite size is larger than the cell size, mark all cells covered by the sprite as obstacles
            var _width  = sprite_width div other.cell_size;
            var _height = sprite_height div other.cell_size;

            for (var _x = 0; _x < _width; _x++)
            {
                for (var _y = 0; _y < _height; _y++)
                {
                    var _gx, _gy;
                    _gx         = (x + (_x * other.cell_size)) div other.cell_size;
                    _gy         = (y + (_y * other.cell_size)) div other.cell_size;

                    other.grid[# _gx, _gy]    = object_index; 
                }
            }
        }
    }
}

/// @desc                       Checks if the given cell coordinates are valid (not an obstacle) in the grid.
/// @param {Real}               _x X coordinate of the cell to check.
/// @param {Real}               _y Y coordinate of the cell to check.
/// @param {id.dsgrid<any*>}    grid The grid to check the cell in.
/// @returns {bool}             Returns true if the cell is valid (not an obstacle), false otherwise.
function _free_path(_struct, _gx, _gy)
{
    gml_pragma("forceinline");    
    return _struct.grid[# _gx, _gy] == 0;
}

/// @desc                       Checks if the given coordinates are valid (not an obstacle) in the grid.
/// @param {Real}               _x X coordinate of the cell to check.
/// @param {Real}               _y Y coordinate of the cell to check.
/// @param {id.dsgrid<any*>}    grid The grid to check the cell in.
/// @returns {bool}             Returns true if the cell is valid (not an obstacle), false otherwise.
function _is_valid_cell(_x, _y, _grid)
{
    gml_pragma("forceinline");
    var _gx     = _x div cell_size;
    var _gy = _y div cell_size;

    return _grid[# _gx, _gy] == 0;
}

/// @desc                   Checks if the given cell coordinates are within the bounds of the grid.
/// @param {Real}           _row Row coordinate of the cell to check.
/// @param {Real} _col      _col Column coordinate of the cell to check.
/// @param {id.dsgrid}      _grid The grid to check the cell in.
/// @returns {bool}         Returns true if the cell is within the bounds of the grid, false otherwise.
function _is_room_cell(_row, _col, _grid)
{
    gml_pragma("forceinline");
    return _row >= 0 && _row < ds_grid_width(_grid) && _col >= 0 && _col < ds_grid_height(_grid);
}

/// @desc                   Returns the actual grid coordinates of the given instance based on its position and cell size.
/// @param {any*}           _struct The structure containing data for the A* algorithm, including the instance and cell size.
/// @returns {struct}        Returns a structure containing the grid coordinates of the instance.
function _actual_location(_struct)
{
    gml_pragma("forceinline");
    var _gx     = _struct.instance.x div _struct.cell_size;
    var _gy     = _struct.instance.y div _struct.cell_size;

    debug_message("Actual location of instance: (" + string(_gx) + ", " + string(_gy) + ")");

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
    debug_message("Manhattan heuristic: (" + string(x1) + ", " + string(y1) + ") to (" + string(x2) + ", " + string(y2) + ")");
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
    debug_message("Euclidean heuristic: (" + string(x1) + ", " + string(y1) + ") to (" + string(x2) + ", " + string(y2) + ")");
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
    debug_message("Simple Euclidean heuristic: (" + string(x1) + ", " + string(y1) + ") to (" + string(x2) + ", " + string(y2) + ")");
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
    debug_message("Octile heuristic: (" + string(x1) + ", " + string(y1) + ") to (" + string(x2) + ", " + string(y2) + ")");
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
    debug_message("Chebyshev heuristic: (" + string(x1) + ", " + string(y1) + ") to (" + string(x2) + ", " + string(y2) + ")");
    return max(dx, dy);
}

#endregion