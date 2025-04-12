extends GDExample


func _on_position_changed(node: Object, new_pos: Vector2) -> void:
	print("Position of " + node.get_class() + "is now: " + str(new_pos))
	pass # Replace with function body.
