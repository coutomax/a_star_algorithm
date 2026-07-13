depth       = 1;

toggle      = true;
move_speed  = 4;

a = a_star(self, obj_target, 32, [obj_wall], manhattan_tie_breaker);
a.on_create();