extends CharacterBody2D

const SPEED = 100.0
const JUMP_VELOCITY = -300.0
const PATH_UPDATE_TIME = 10
const POSITION_TOLERANCE = 2.0;

@onready var pathfinder = get_node("../Pathfinder")
@export var target: Node2D

var current_path;
var path_update_timer = 0.0

var current_action_idx = 0
var in_action = false
var last_pos

func _process(delta: float) -> void:
	if path_update_timer >= 0.0:
		path_update_timer -= delta
	# (and not in the middle of an action)
	elif !in_action:
		path_update_timer = PATH_UPDATE_TIME
		
		var from = position
		var to = target.position
		current_path = pathfinder.find_path(from, to)
		current_action_idx = 0
	
func _physics_process(delta: float) -> void:
	if current_path and current_action_idx < current_path.size():
		var action = current_path[current_action_idx]

		var horiz_direction := 0
		
		var completed_action = false

		if action.is_jump:
			print(action.from)
			print(action.to)
			print(action.jump_velocity)
			
			if !in_action:
				if ((action.from.x <= position.x and action.from.x >= last_pos.x)
				or (action.from.x <= last_pos.x and action.from.x >= position.x)):
					# jumping!
					velocity = action.jump_velocity * 31;
					in_action = true
				else:
					horiz_direction = sign((action.from - position).x)
			else:
				if ((action.to.x <= position.x and action.to.x >= last_pos.x)
				or (action.to.x <= last_pos.x and action.to.x >= position.x)):
					in_action = false
					completed_action = true
		else:
			horiz_direction = sign((action.to - position).x)
			print(last_pos.x)
			print(position.x)
			print(action.to.x)
			print(horiz_direction)
			
			if ((action.to.x <= position.x and action.to.x >= last_pos.x)
			or (action.to.x <= last_pos.x and action.to.x >= position.x)):
				# abs(action.to.x - position.x) < POSITION_TOLERANCE
				completed_action = true
		
		if !in_action:
			if horiz_direction:
				velocity.x = horiz_direction * SPEED
			else:
				velocity.x = move_toward(velocity.x, 0, SPEED)

		if completed_action:
			current_action_idx += 1
	else:
		velocity.x = move_toward(velocity.x, 0, SPEED)

	# Add the gravity.
	if not is_on_floor():
		velocity += get_gravity() * delta
		
	last_pos = position
	
	move_and_slide()
