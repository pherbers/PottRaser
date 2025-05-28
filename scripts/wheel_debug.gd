extends Node3D

var wheel: WheelSuspension
var arrowX: MeshInstance3D
var arrowY: MeshInstance3D
var arrowZ: MeshInstance3D
@export var force_scale: float = 0.001
@export var arrowModel: Mesh
@export var arrowMaterial: Material

func _ready():
    wheel = get_parent() as WheelSuspension
    arrowX = MeshInstance3D.new()
    arrowX.mesh = arrowModel
    arrowX.material_override = arrowMaterial.duplicate()
    add_child(arrowX)
    arrowY = MeshInstance3D.new()
    arrowY.mesh = arrowModel
    arrowY.material_override = arrowMaterial.duplicate()
    add_child(arrowY)
    arrowZ = MeshInstance3D.new()
    arrowZ.mesh = arrowModel
    arrowZ.material_override = arrowMaterial.duplicate()
    add_child(arrowZ)

    arrowX.material_override.set("albedo_color", Color.RED)
    arrowY.material_override.set("albedo_color", Color.GREEN)
    arrowZ.material_override.set("albedo_color", Color.BLUE)


func _physics_process(delta):

    if wheel._last_force_debug.x.length() > 0.1:
        arrowX.global_position = wheel.wheel_viz.global_position
        arrowX.global_transform = arrowX.global_transform.looking_at(wheel.wheel_viz.global_position + wheel._last_force_debug.x, Vector3.UP, true)
    if wheel._last_force_debug.y.length() > 0.1:
        arrowY.global_position = wheel.wheel_viz.global_position
        arrowY.global_transform = arrowY.global_transform.looking_at(wheel.wheel_viz.global_position + wheel._last_force_debug.y, Vector3.RIGHT, true)
    if wheel._last_force_debug.z.length() > 0.1:
        arrowZ.global_position = wheel.wheel_viz.global_position
        arrowZ.global_transform = arrowZ.global_transform.looking_at(wheel.wheel_viz.global_position + wheel._last_force_debug.z, Vector3.UP, true)

    arrowX.scale = Vector3.ONE * pow(wheel._last_force_debug.x.length() * force_scale, .5)
    arrowY.scale = Vector3.ONE * pow(wheel._last_force_debug.y.length() * force_scale, .5)
    arrowZ.scale = Vector3.ONE * pow(wheel._last_force_debug.z.length() * force_scale, .5)

    if wheel._wheel_state == WheelSuspension.WheelState.SLIDE:
        arrowX.material_override.set("albedo_color", Color.PALE_VIOLET_RED)
    else:
        arrowX.material_override.set("albedo_color", Color.RED)
