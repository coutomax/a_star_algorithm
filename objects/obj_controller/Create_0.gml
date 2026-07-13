toggle      = false;
move_speed  = 4;

a = a_star(obj_enemy, obj_target, 32, [obj_wall], manhattan_tie_breaker);
a.on_create();