extends Node3D

class_name WheelSuspension

enum WheelState {
    GRIP, SLIDE, OFFGROUND,
}

@export_category("Wheels")
@export var wheel_viz: Node3D
@export var wheel_radius = 0.5
@export var wheel_width = 0.2
@export var wheel_friction_drag = 100
@export var wheel_grip_friction_coeff = 1.0
@export var wheel_slide_friction_coeff = 0.6
@export var steering: bool = false

@export_category("Suspension")
@export var spring_length = 0.3
@export var spring_force = 6000
@export var suspension_dampening = 0.5
@export var bump = 0.1
@export var rebound = 0.1

var _wheel_compression: float = 1.
var _wheel_state: WheelState = WheelState.GRIP
var _last_force_debug: Basis
