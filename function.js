"use strict";
function string_to_float_array(input){
	var a = input.trim().split(' ');
	for(var i = 0; i < a.length ; ++i){
		a[i] = parseFloat(a[i]);
	}
	return a;
}
function get_next_line(a){
	while(a.length > 0){
		var i = a.shift();
		if(i[0] !== '#'){
			return i;
		}
	}
}
function canvas_coordinate(pos,x_scale,y_scale,y_height){
	return [pos[0]*x_scale, y_height - pos[1]*y_scale];
}
function origin_coordinate(pos,x_scale,y_scale,y_height){
	return [pos[0]/x_scale, (y_height - pos[1])/y_scale];
}
function origin_distance(pos,x_scale,y_scale){
	return [pos[0]/x_scale,  -1*pos[1]/y_scale];
}
function line_intersect(a1,a2,b1,b2){
	function not_intersect1D(n1,n2,m1,m2){
		if(n1 > n2){
			n2 = [n1, n1 = n2][0]; //swap
		}
		if(m1 > m2){
			m2 = [m1, m1 = m2][0]; //swap
		}
		return Math.max(n1,m1) > Math.min(n2,m2);
	}
	function cross(i,a,b){
		return (a[0] - i[0])*(b[1] - i[1]) - (a[1] - i[1])*(b[0] - i[0]);
	}
	if(not_intersect1D(a1[0],a2[0],b1[0],b2[0])){
		return false;
	}
	if(not_intersect1D(a1[1],a2[1],b1[1],b2[1])){
		return false;
	}
	var v0 = cross(a1,a2,b1);
	var v1 = cross(a1,a2,b2);
	if(v0 !== 0 && v1 !==0){
		if((v0 > 0) ^ (v1 < 0)){
			return false;
		}
	}
	else{
		return false;
	}
	v0 = cross(b1,b2,a1);
	v1 = cross(b1,b2,a2);
	if(v0 !== 0 && v1 !==0){
		if((v0 > 0) ^ (v1 < 0)){
			return false;
		}
	}
	else{
		return false;
	}
	return true;
}
function angle_calculator(base,p){
	return Math.atan2(p[1], p[0]) - Math.atan2(base[1],base[0]);
}
function Array2D(x_size, y_size){
	this.width = Math.round(x_size);
	this.height = Math.round(y_size);
	this.arr = new Array(this.width*this.height);
	this.arr.fill(null);
}
Array2D.prototype.to_index = function(p){
	return this.width*Math.round(p[1])+Math.round(p[0]);
};
Array2D.prototype.my_size = function(){
	return [this.width,this.height];
};
Array2D.prototype.get_value = function(p){
	var i = this.to_index(p);
	if(i >= this.arr.length || i < 0){
		return null;
	}
	return this.arr[i];
};
Array2D.prototype.set_value = function(p,v){
	this.arr[this.to_index(p)] = v;
};
Array2D.prototype.vaild = function(p){
	if(p[0] < this.width && p[1] < this.height){
		return p[0] >= 0 && p[1] >= 0;
	}
	return false;
}
Array2D.prototype.is_null = function(p){
	if(this.vaild(p)){
		return this.arr[this.to_index(p)] === null;
	}
	return false;
};
Array2D.prototype.local_min = function(p){
	var tmp = [];
	var my_list = [[0,1],[0,-1],[1,0],[-1,0]];
	for(var i = 0 ; i< my_list.length ; ++i){
		var offset = my_list[i];
		var local = [p[0]+offset[0], p[1]+offset[1]];
		if(this.vaild(local)){
			var v = this.arr[this.to_index(local)];
			if(v !== null && v !== -1){
				tmp.push(v);
			}
		}
	}
	if(tmp.length === 0){
		return -1;
	}
	return Math.min(...tmp);
};
function RunningMeta(robot, obstacles){
	this.robot = robot;
	this.obstacles = obstacles;
	this.path = [];
	this.values = {};
}
RunningMeta.prototype.to_index = function(pos){
	var tmp = '';
	pos.forEach(function(item){
		tmp += (item.toString()+',');
	});
	return tmp;
};
RunningMeta.prototype.in_path = function(pos){
	for(var i = this.path.length -1; i >= 0 ; --i){
		var item = this.path[i];
		if(item[0] === pos[0] && item[1] === pos[1] && item[2] === pos[2]){
			return true;
		}
	}
	return false;
};
RunningMeta.prototype.get_position_value = function(pos){
	if(this.in_path(pos)){
		return null;
	}
	var i = this.to_index(pos);
	if(this.values.hasOwnProperty(i)){
		return this.values[i];
	}
	var v = this.robot.status_value(pos, this.obstacles);
	this.values[i] = v;
	return v;
};