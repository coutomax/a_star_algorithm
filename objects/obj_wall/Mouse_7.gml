global.grid_properties.remove_instance(id);
global.grid = variable_clone(global.grid_properties);

instance_destroy(self);