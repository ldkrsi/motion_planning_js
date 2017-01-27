"use strict";

var scope = new CanvasPanel();
$.getJSON('map.json', function(data){
	scope.init_obstacles(
		data['obstacles'].map(function(item){
			return new Obstacle(item['polygons'].map(function(p){
				return new Polygon(p);
			}), item['init_config']);
		})
	);
	scope.draw_obstacles_layer();
	var robot = data['robot'];
	scope.init_robots(new Robot(robot['polygons'].map(function(p){
		return new Polygon(p);
	}), robot['init_config'], robot['goal_config'], robot['control_points']));
	scope.draw_robots_layer();
	scope.draw_goals_layer();
	scope.make_potential_fields();
});

$(document).on('mousedown', scope.touch_panel.selector, function(e){
	var result = scope.onmousedown(e.offsetX,e.offsetY);
	if(result === null){
		return;
	}
	scope.mouse_event.rotate = $('#rotate').prop('checked');
	scope.mouse_event.add_click_point(e.offsetX, e.offsetY, scope.mouse_event.object);
	scope.mouse_event.interval_id = setInterval(function(){
		result.call(scope,false);
	}, 1000 / 30.0);
});
$(document).on('mouseup', scope.touch_panel.selector, function(e){
	if(scope.mouse_event.interval_id === null){
		return;
	}
	clearInterval(scope.mouse_event.interval_id);
	scope.onmouseup();
	
});
$(document).on('mousemove', scope.touch_panel.selector, function(e){
	if(scope.mouse_event.interval_id === null){
		return;
	}
	if(scope.mouse_event.rotate){
		scope.rotate_object(e.offsetX,e.offsetY);
	}else{
		scope.move_object(e.offsetX,e.offsetY);
	}
});

$(document).on('click','.potential-field-button',function(e){
	var j = $(e.target).data('controlPointIndex');
	scope.show_potential_fields(j);
});
$(document).on('click','button.run',function(e){
	scope.get_solution();
});
