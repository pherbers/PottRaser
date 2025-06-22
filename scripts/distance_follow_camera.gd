extends Node3D

class_name DistanceFollowCamera

@export var follow_target: Node3D
@export var distance: float = 5.
@export var height: float = 2.
@onready var camera: PhantomCamera3D = $PhantomCamera3D

func _init():
    pass

func _process(delta):
    var follow_vec = follow_target.global_position + Vector3.UP * height
    var delta_vec = (global_position - follow_vec).slide(Vector3.UP)
    var target_pos = delta_vec.normalized() * distance + follow_vec
    global_position = target_pos
