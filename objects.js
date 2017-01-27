"use strict";
function Polygon(vertices){
	this.vertices = vertices;
	this.path_cached = null;
};
Polygon.prototype.clone = function(){
	var p = new Polygon();
	p.vertices = this.vertices.slice();
	p.path_cached = null;
	return p;
};
Polygon.prototype.in_polygon = function(x,y){
	var path = this.path();
	var new_array = [path[path.length-1]].concat(path);
	var flag = false;
	for(var i = 1; i < new_array.length; ++i){
		var p1 = new_array[i];
		var p2 = new_array[i-1];
		if((p1[1] > y) === (p2[1] > y)){
			continue;
		}
		if(x < (p2[0]-p1[0])*(y-p1[1])/(p2[1]-p1[1])+p1[0]){
			flag = !flag;
		}
	}
	return flag;
};
Polygon.prototype.line_intersect = function(s,t){
	var path = this.path();
	var lines1 = [path[path.length-1]].concat(path);
	for(var i = 1;i < lines1.length; ++i){
		var a1 = lines1[i-1];
		var a2 = lines1[i];
		if(line_intersect(a1,a2,s,t)){
			return true;
		}
	}
	if(this.in_polygon.apply(this, s)){
		return true;
	}
	return false;
}
Polygon.prototype.polygon_intersect = function(poly){
	var path = this.path();
	var lines1 = [path[path.length-1]].concat(path);
	path = poly.path();
	var lines2 = [path[path.length-1]].concat(path);
	for(var i = 1;i < lines1.length; ++i){
		var a1 = lines1[i-1];
		var a2 = lines1[i];
		for(var j = 1;j < lines2.length; ++j){
			var b1 = lines2[j-1];
			var b2 = lines2[j];
			if(line_intersect(a1,a2,b1,b2)){
				return true;
			}
		}
	}
	if(poly.in_polygon.apply(poly, lines1[0])){
		return true;
	}
	return this.in_polygon.apply(this, lines2[0]);
};
Polygon.prototype.path = function(offset_x, offset_y, theta, cache){
	if(cache === undefined){
		cache = true;
	}
	if(this.path_cached !== null && cache === true){
		return this.path_cached.slice();
	}
	this.path_cached = [];
	var ans = this.path_cached;
	this.vertices.forEach(function(v){
		var cos = Math.cos(theta);
		var sin = Math.sin(theta);
		var x = cos*v[0] - sin*v[1] + offset_x;
		var y = sin*v[0] + cos*v[1] + offset_y;
		ans.push([x,y]);
	});
	return ans.slice();
};
Polygon.prototype.path_in_point = function(){
	var points = [];
	var path = this.path();
	var lines = [path[path.length-1]].concat(path);
	for(var i = 1;i < lines.length; ++i){
		var a1 = lines[i-1];
		var a2 = lines[i];
		var d = Math.max(Math.abs(a1[0] - a2[0]), Math.abs(a1[1] - a2[1]));
		var dx = (a1[0] - a2[0])/d;
		var dy = (a1[1] - a2[1])/d;
		for(var j = 0;j < d; ++j){
			points.push([Math.round(a2[0]+j*dx), Math.round(a2[1]+j*dy)]);
		}
	}
	return points;
}

function Obstacle(polys, config){
	this.polygons = polys;
	this.init_config = config['position'];
	this.init_config.push(config['theta']*Math.PI/180);
};
Obstacle.prototype.copy = function(){
	var tmp = new Obstacle();
	tmp.polygons = this.polygons.slice();
	tmp.init_config = this.init_config.slice();
	return tmp;
};
Obstacle.prototype.inside = function(x,y){
	for(var i=0; i < this.polygons.length; ++i){
		if(this.polygons[i].in_polygon(x,y)){
			return true;
		}
	}
	return false;
};
Obstacle.prototype.intersect = function(obs){
	for(var i=0; i < this.polygons.length; ++i){
		var p = this.polygons[i];
		for(var j=0; j < obs.polygons.length; ++j){
			if(p.polygon_intersect(obs.polygons[j])){
				return true;
			}
		}
	}
	return false;
}
Obstacle.prototype.paths = function(cache){
	var ans = [];
	var params = this.init_config.slice();
	params.push(cache);
	this.polygons.forEach(function(p){
		ans.push(p.path.apply(p, params));
	});
	return ans;
};
Obstacle.prototype.calculate_angle = function(base, p){
	return angle_calculator(base,
		[p[0] - this.init_config[0], p[1] - this.init_config[1]]
	);
};
Obstacle.prototype.transform_point = function(p){
	var theta = this.init_config[2];
	var cos = Math.cos(theta);
	var sin = Math.sin(theta);
	var x = cos*p[0] - sin*p[1] + this.init_config[0];
	var y = sin*p[0] + cos*p[1] + this.init_config[1];
	return [x,y];
};
Obstacle.prototype.set_init_config = function(changes){
	this.init_config[0] += changes[0];
	this.init_config[1] += changes[1];
	this.init_config[2] += (changes[2]*Math.PI/180);
}

function Robot(polys, init, goal, ctrl_points){
	this.init = new Obstacle(polys, init);
	this.goal = new Obstacle(polys.map(function(p){
		return p.clone();
	}), goal);
	this.control_points = ctrl_points;
	this.potential_fields = Array(this.control_points.length).fill(null);
};
Robot.prototype.get_goal_control_points = function(){
	var result = [];
	for(var i=0;i<this.control_points.length;++i){
		var point = this.control_points[i];
		result.push(this.goal.transform_point(point));
	}
	return result;
};
Robot.prototype.get_now_control_points = function(){
	var result = [];
	for(var i=0;i<this.control_points.length;++i){
		var point = this.control_points[i];
		result.push(this.init.transform_point(point));
	}
	return result;
};
Robot.prototype.status_value = function(changes,obs){
	var origin = this.init.init_config.slice();
	this.init.init_config[0] += changes[0];
	this.init.init_config[1] += changes[1];
	this.init.init_config[2] += (changes[2]*Math.PI/180);
	this.init.paths(false);
	for(var i = 0; i < obs.length; ++i){
		if(this.init.intersect(obs[i])){
			this.init.init_config = origin.slice();
			return null;
		}
	}
	var ps = this.get_now_control_points();
	var sum = 0;
	for(var i = 0 ; i < this.potential_fields.length ; ++i){
		var v = this.potential_fields[i].get_value(ps[i]);
		if(v === null || v < 0){
			this.init.init_config = origin.slice();
			return null;
		}
		sum += v;
	}
	this.init.init_config = origin.slice();
	//this.init.paths(false);
	return sum;
};
Robot.prototype.set_init_config = function(changes){
	this.init.set_init_config(changes);
};

function Layer(dom_id,color,config){
	this.map_size = config['map_size'];
	this.view_size = config['view_size'];
	this.x_scale = config['x_scale'];
	this.y_scale = config['y_scale'];
	this.color = color;
	this.entities = [];
	this.canvas = document.getElementById(dom_id);
	this.ctx = this.canvas.getContext('2d');
	this.canvas.width = this.view_size[0];
	this.canvas.height = this.view_size[1];
};
Layer.prototype.draw = function(cache){
	var parent = this;
	var draw_path_function = parent.draw_by_path;
	parent.ctx.clearRect(0,0,this.view_size[0],this.view_size[1]);
	parent.entities.forEach(function(obj){
		obj.paths(cache).forEach(function(p){
			draw_path_function.call(parent,p);
		});
	});
};
Layer.prototype.draw_by_path = function(path){
	var c = this.ctx;
	var x_scale = this.x_scale;
	var y_scale = this.y_scale;
	var y_height = this.view_size[1];
	c.fillStyle = this.color;
	c.strokeStyle = this.color;
	c.lineWidth = 1;
	c.beginPath();
	c.moveTo.apply(c,canvas_coordinate(path.shift(),x_scale,y_scale,y_height));
	path.forEach(function(p){
		c.lineTo.apply(c,canvas_coordinate(p,x_scale,y_scale,y_height));
	});
	c.closePath();
	c.fill();
	c.stroke();
};

function PotentialLayer(config){
	this.map_size = config['map_size'];
	this.view_size = config['view_size'];
	this.x_scale = config['x_scale'];
	this.y_scale = config['y_scale'];
	this.canvas = document.getElementById(config['layer']['potential_layer']);
	this.ctx = this.canvas.getContext('2d');
	this.canvas.width = this.view_size[0];
	this.canvas.height = this.view_size[1];
};
PotentialLayer.prototype.draw_point = function(p,v){
	var new_p = canvas_coordinate(p, this.x_scale, this.y_scale, this.view_size[1]);
	var c = v.toString();
	this.ctx.fillStyle = 'rgb('+c+','+c+','+c+')';
	this.ctx.strokeStyle = this.ctx.fillStyle;
	this.ctx.lineWidth = 1;
	var x = new_p[0] - this.x_scale/2;
	var y = new_p[1] - this.y_scale/2;
	this.ctx.fillRect(x, y, this.x_scale, this.y_scale);
	this.ctx.strokeRect(x, y, this.x_scale, this.y_scale);
}
PotentialLayer.prototype.clear = function(){
	this.ctx.fillRect(0, 0, this.view_size[0], this.view_size[1]);
}

function MouseEvent(){
	this.reset();
};
MouseEvent.prototype.reset = function(){
	this.rotate = false;
	this.interval_id = null;
	this.click_point = null;
	this.click_point_origin = null;
	this.object = null;
	this.obj_init_config = null;
	this.draw_function = null;
	this.obj_type = null;
};
MouseEvent.prototype.set_objcet = function(obj,type){
	this.object = obj;
	this.obj_init_config = obj.init_config.slice();
	this.obj_type = type;
};
MouseEvent.prototype.add_click_point = function(x,y,obj){
	this.click_point = [x,y];
	var new_point = origin_coordinate(this.click_point,
		globe_config['x_scale'], globe_config['y_scale'], globe_config['view_size'][1]
	);
	this.click_point_obj = [new_point[0] - obj.init_config[0], new_point[1] - obj.init_config[1]];
};

function TouchLayer(dom_id, config){
	this.view_size = config['view_size'];
	this.selector = '#' + dom_id;
	this.canvas = document.getElementById(dom_id);
	this.ctx = this.canvas.getContext('2d');
	this.canvas.width = this.view_size[0];
	this.canvas.height = this.view_size[1];
};

function CanvasPanel(){
	this.touch_panel = new TouchLayer(globe_config['layer']['touch_layer'], globe_config);
	this.mouse_event = new MouseEvent();
	this.potential = new PotentialLayer(globe_config);
};
CanvasPanel.prototype.init_robots = function(robot){
	this.robot = robot;
	this.robots_layer = null;
	this.goals_layer = null;
};
CanvasPanel.prototype.boundary_obs = function(){
	var p0 = new Polygon([
		[0.0, 0.0],
		[globe_config['map_size'][0], 0.0],
		[globe_config['map_size'][0], -1.0],
		[-1.0, -1.0],
		[-1.0, globe_config['map_size'][1]],
		[0.0, globe_config['map_size'][1]]
	]);
	var p1 = new Polygon([
		[0.0, globe_config['map_size'][1]],
		[0.0, globe_config['map_size'][1]+1.0],
		[globe_config['map_size'][0]+1.0, globe_config['map_size'][1]+1.0],
		[globe_config['map_size'][0]+1.0, 0],
		[globe_config['map_size'][0], 0.0],
		[globe_config['map_size'][0], globe_config['map_size'][1]],
	]);
	return new Obstacle([p0, p1],{
		'position': [.0,.0],
		'theta': .0
	});
}
CanvasPanel.prototype.init_obstacles = function(obs_arr){
	this.obstacles = [this.boundary_obs()];
	this.obstacles.push.apply(this.obstacles, obs_arr);
	this.obstacles_layer = null;
};
CanvasPanel.prototype.draw_robots_layer = function(cache){
	if(this.robots_layer === null){
		this.robots_layer = new Layer('robots','#4caf50',globe_config);
		this.robots_layer.entities.push(this.robot.init);
	}
	this.robots_layer.draw(cache);
};
CanvasPanel.prototype.draw_goals_layer = function(cache){
	if(this.goals_layer === null){
		this.goals_layer = new Layer('goals','#2196f3',globe_config);
		this.goals_layer.entities.push(this.robot.goal);
	}
	this.goals_layer.draw(cache);
};
CanvasPanel.prototype.draw_obstacles_layer = function(cache){
	if(this.obstacles_layer === null){
		this.obstacles_layer = new Layer('obstacles','#f44336',globe_config);
		this.obstacles_layer.entities = this.obstacles;
	}
	this.obstacles_layer.draw(cache);
};
CanvasPanel.prototype.intersect_evnet = function(){
	if(this.mouse_event.obj_type === 'init' || this.mouse_event.obj_type === 'goal'){
		for(var i = 0; i < this.obstacles.length; ++i){
			if(this.mouse_event.object.intersect(this.obstacles[i])){
				return true;
			}
		}
	}
	else if(this.mouse_event.obj_type === 'obstacle'){
		if(this.mouse_event.object.intersect(this.robot.goal) || this.mouse_event.object.intersect(this.robot.init)){
			return true;
		}
	}
	return false;
}
CanvasPanel.prototype.onmousedown = function(x,y){
	var pos = origin_coordinate([x,y], globe_config['x_scale'], globe_config['y_scale'], globe_config['view_size'][1]);
	var r = this.robot;
	if(r.init.inside.apply(r.init, pos)){
		this.mouse_event.set_objcet(r.init,'init');
		return this.mouse_event.draw_function = this.draw_robots_layer;
	}
	if(r.goal.inside.apply(r.goal, pos)){
		this.mouse_event.set_objcet(r.goal,'goal');
		return this.mouse_event.draw_function = this.draw_goals_layer;
	}
	for(var i = 0; i < this.obstacles.length; ++i){
		var obs = this.obstacles[i];
		if(obs.inside.apply(obs, pos)){
			this.mouse_event.set_objcet(obs,'obstacle');
			return this.mouse_event.draw_function = this.draw_obstacles_layer;
		}
	}
	return null;
};
CanvasPanel.prototype.onmouseup = function(){
	this.mouse_event.interval_id = null;
	if(this.intersect_evnet()){
		this.mouse_event.object.init_config = this.mouse_event.obj_init_config.slice();
	}
	else{
		this.make_potential_fields();
	}
	this.mouse_event.draw_function.call(this,false);
	this.mouse_event.reset();
}
CanvasPanel.prototype.move_object = function(x,y){
	var values = origin_distance(
		[x - this.mouse_event.click_point[0], y - this.mouse_event.click_point[1]], 
		globe_config['x_scale'], globe_config['y_scale']
	);
	this.mouse_event.object.init_config[0] = values[0] + this.mouse_event.obj_init_config[0];
	this.mouse_event.object.init_config[1] = values[1] + this.mouse_event.obj_init_config[1];
};
CanvasPanel.prototype.rotate_object = function(x,y){
	var theta = this.mouse_event.object.calculate_angle(this.mouse_event.click_point_obj,
		origin_coordinate([x,y],
			globe_config['x_scale'],globe_config['y_scale'], globe_config['view_size'][1]
		)
	);
	this.mouse_event.object.init_config[2] = theta + this.mouse_event.obj_init_config[2];
};
CanvasPanel.prototype.make_potential_fields = function(){
	var r = this.robot;
	var points = r.get_goal_control_points();
	for(var j = 0 ; j<points.length ; ++j){
		var start = [Math.round(points[j][0]), Math.round(points[j][1])];
		var arr = new Array2D(globe_config['map_size'][0], globe_config['map_size'][1]);
		var queue = [start];
		for(var k = 0; k < this.obstacles.length; ++k){
			this.obstacles[k].polygons.forEach(function(poly){
				poly.path_in_point().forEach(function(_p){
					if(_p[0] < globe_config['map_size'][0] && _p[0] < globe_config['map_size'][1]){
						arr.set_value(_p, -1);
					}
				});
			});
		}
		while(queue.length > 0){
			var p = queue.shift();
			if(!arr.is_null(p)){
				continue;
			}
			arr.set_value(p, arr.local_min(p)+1);
			[[0,-1],[0,1],[1,0],[-1,0]].forEach(function(offset){
			var new_p = [p[0]+offset[0], p[1]+offset[1]];
				if(arr.is_null(new_p)){
					queue.push(new_p);
				}
			});
		}
		r.potential_fields[j] = arr;
		r.paths = [];
	}
	this.make_potential_fields_button();
};
CanvasPanel.prototype.make_potential_fields_button = function(){
	$('.potential-fields').empty();
	var r = this.robot;
	for(var j = 0 ; j<r.control_points.length ; ++j){
		var btn = $('<button></button>').html(j.toString())
			.attr('data-control-point-index', j).addClass('potential-field-button');
		$('.potential-fields').append(btn);
	}
	$('.potential-fields').append('<br/>');
};
CanvasPanel.prototype.show_potential_fields = function(j){
	var target = this.robot.potential_fields[j];
	var size = target.my_size();
	this.potential.clear();
	for(var y = 0; y < size[1] ; ++y){
		var tmp = target.arr.slice(y*size[0], (y+1)*size[0]);
		for(var x = 0; x < size[0] ; ++x){
			var value = 255;
			if(tmp[x] !== null && tmp[x] !== -1){
				value = Math.min(tmp[x],255);
			}
			this.potential.draw_point([x,y],value);
		}
	}
};
CanvasPanel.prototype.get_solution2 = function(){
	for(var i = 0; i< this.robots.length; ++i){
		var item = this.robots[i];
		this.get_solution_inner(item);
	}
}
CanvasPanel.prototype.get_solution = function(){
	var meta_data = new RunningMeta(this.robot, this.obstacles);
	var init_config = this.robot.init.init_config.slice();
	function searching(now_at){
		var order_list = [];
		globe_config['search_directs'].forEach(function(next){
			var tmp = [0,0,0];
			tmp[0] = now_at[0] + next[0];
			tmp[1] = now_at[1] + next[1];
			tmp[2] = (now_at[2] + next[2]) % 360;
			if(tmp[2] >= 180){
				tmp[2] -= 360;
			}
			else if(tmp[2] < -180){
				tmp[2] += 360;
			}
			var v = meta_data.get_position_value(tmp);
			if(v === null){
				return;
			}
			order_list.push([tmp, v]);
		});
		order_list.sort(function(a, b){
			return a[1] - b[1];
		});
		meta_data.path.push(now_at);
		for(var i = 0;i < order_list.length; ++i){
			var item = order_list[i];
			if(item[1] === 0){
				meta_data.path.push(item[0]);
				return true;
			}
			var result = searching(item[0]);
			if(result === true){
				return true;
			}
		}
		meta_data.path.pop();
		return false;
	}
	try{
		var ans = searching([0,0,0]);
	}
	catch(e){
		var ans = false;
		this.robot.init.init_config = init_config.slice();
		this.draw_robots_layer(false);
	}
	
	console.log(ans);
	if(ans){
		$('#result').html('success');
		this.playing(this.robot, meta_data.path);
	}
	else{
		$('#result').html('failure');
	}
};
CanvasPanel.prototype.playing = function(robot, paths){
	var parent = this;
	var init_config = robot.init.init_config.slice();
	var id = setInterval(function(){
		var c = paths.shift();
		robot.set_init_config(c);
		parent.draw_robots_layer(false);
		if(paths.length === 0){
			clearInterval(id);
		}else{
			robot.init.init_config = init_config.slice();
		}
	}, 1000 / 45.0);
};
