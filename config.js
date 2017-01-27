"use strict";
var globe_config = {
	'map_size': [128.0, 128.0],
	'view_size': [400.0, 400.0],
	'layer': {
		'touch_layer': 'touch-layer',
		'potential_layer': 'potential'
	},
	'search_directs': [[1,0,0], [0,1,0], [-1,0,0], [0,-1,0], [0,0,2], [0,0,-2]]
};
(function(){
	globe_config['x_scale'] = globe_config['view_size'][0]/globe_config['map_size'][0];
	globe_config['y_scale'] = globe_config['view_size'][1]/globe_config['map_size'][1];
})();