extends RigidBody3D

class_name Car
# Trying to build a raycast suspension
# Ideas from https://lupine-vidya.itch.io/gdsim/devlog/677580/workshop-i-suspending-a-rigidbody-mirror
@export var viewmodel: Node3D
@export_category("Controls")
@export var acceleration = 100.0
@export var max_rpm = 6000
@export var wheel_translation = 0.01
@export var brake_force = 1000.0

@export_category("Wheels and Suspension")
@export var wheels: Array[WheelSuspension]

@export var motor_curve: Curve

func _ready():
    assert(len(wheels) > 0, "Car has no wheels...")
    for whl in wheels:
        assert(whl.wheel_viz != null, "No Wheel visualization for car!")
    # testing...
    apply_central_impulse(transform.basis.x * 7 + transform.basis.z * 2)

func _physics_process(delta):
    var space_state = get_world_3d().direct_space_state

    ## Number of wheels touching the ground to calculate force distribution
    var n_wheel_contacts = 0
    for whl in wheels:
        if whl._wheel_state != WheelSuspension.WheelState.OFFGROUND:
            n_wheel_contacts += 1
    if n_wheel_contacts == 0:
        n_wheel_contacts = 1  # to avoid div by zero

    for whl in wheels:
        var s = whl.position
        var w = whl.wheel_viz

        var is_steering = whl.steering
        var spring_length = whl.spring_length
        var spring = whl.spring_force
        var wheel_radius = whl.wheel_radius

        var sg = to_global(s)
        var rquery = PhysicsRayQueryParameters3D.create(sg, to_global(s + Vector3.DOWN * (spring_length + wheel_radius)), collision_mask)
        var ray_results = space_state.intersect_ray(rquery)
        var distance = spring_length
        if ray_results:
            var hit_pos: Vector3 = ray_results["position"]
            distance = hit_pos.distance_to(sg) - wheel_radius

        var compress = 1 - (distance / spring_length)  # 1 is fully compressed
        var y_force = spring * compress * spring_length

        var prev_compress = whl._wheel_compression
        var damping = whl.suspension_dampening * (compress - prev_compress) / delta * spring
        y_force += damping
        whl._wheel_compression = compress

        if (compress - prev_compress) >= 0:
            y_force += whl.bump * (compress - prev_compress) / delta
        else:
            y_force += whl.rebound * (compress - prev_compress) / delta

        var force_point = sg - global_transform.origin

        var normal = Vector3.ZERO
        if ray_results:
            normal = ray_results["normal"]

        var basis_x = transform.basis.x
        var basis_z = transform.basis.z
        if whl.steering:
            basis_x = basis_x.rotated(transform.basis.y, -deg_to_rad(steering * max_steering_angle))
            basis_z = basis_z.rotated(transform.basis.y, -deg_to_rad(steering * max_steering_angle))


        var normal_up = normal.normalized()
        var forward = basis_x.cross(normal).normalized()
        var left = -basis_z.cross(normal).normalized()

        var normal_basis = Basis(left, normal_up, forward)
        #print(normal_basis, normal_basis.x.cross(normal_basis.z))

        var spring_force = y_force
        var spring_force_vec = normal * spring_force

        var traction_force_z = Vector3.ZERO
        var traction_force_x = Vector3.ZERO

        var point_velocity = get_point_velocity(sg)
        var velocity_local_x = normal_basis.tdotx(point_velocity)
        var velocity_local_y = normal_basis.tdoty(point_velocity)
        var velocity_local_z = normal_basis.tdotz(point_velocity)

        if compress > 0:
            # wheel is on the ground, apply friction
            var slip: float
            var force_x = velocity_local_x * mass / delta / n_wheel_contacts# + (normal_basis.tdotx(get_gravity()) * mass / n_wheel_contacts))
            var force_z = abs(velocity_local_z * mass / delta / n_wheel_contacts)
            var friction_force_x = 0
            var grav_force = normal_basis.tdoty(get_gravity()) * mass / n_wheel_contacts
            if abs(velocity_local_x) < 1:
                # Regain grip
                if(whl._wheel_state == WheelSuspension.WheelState.SLIDE):
                    print(whl.name, ": Regained Grip")
                whl._wheel_state = WheelSuspension.WheelState.GRIP
            if (whl._wheel_state == WheelSuspension.WheelState.GRIP and
                abs(force_x) < abs(grav_force * 60 * whl.wheel_grip_friction_coeff)):
                # still gripping
                slip = whl.wheel_grip_friction_coeff
                friction_force_x = force_x
                whl._wheel_state = WheelSuspension.WheelState.GRIP
            else:
                # sliding
                slip = whl.wheel_slide_friction_coeff
                friction_force_x = abs(grav_force) * slip * sign(force_x)
                if(whl._wheel_state == WheelSuspension.WheelState.GRIP):
                    print(whl.name, ": Lost Grip, ", abs(force_x), "\t", y_force * whl.wheel_grip_friction_coeff)
                whl._wheel_state = WheelSuspension.WheelState.SLIDE

            traction_force_x = -left * friction_force_x

            var zSlip = whl.wheel_grip_friction_coeff
            if whl._wheel_state == WheelSuspension.WheelState.SLIDE:
                zSlip = whl.wheel_slide_friction_coeff
            # Wheel torque
            if braking > 0:
                var bremsfaktor = 0.05
                var brakeAlignment = forward.normalized().dot(linear_velocity.normalized())
                traction_force_z = forward * velocity_local_z / delta * mass / n_wheel_contacts * -1 * zSlip * bremsfaktor * brakeAlignment
            elif accelerate > 0 and not is_steering:
                var wheel_torque = motor_curve.sample_baked(rpm/max_rpm) * max_rpm * wheel_translation
                traction_force_z = forward * wheel_torque * zSlip
            else:
                if(abs(velocity_local_z) > 0.01):
                    traction_force_z = -sign(velocity_local_z) * forward * whl.wheel_friction_drag
                else:
                    traction_force_z = Vector3.ZERO
        else:
            whl._wheel_state = WheelSuspension.WheelState.OFFGROUND


        apply_force(traction_force_x + traction_force_z + spring_force_vec, force_point)
        whl._last_force_debug = Basis(traction_force_x, spring_force_vec, traction_force_z)
        #print(traction_force_x.length(), "\t", spring_force_vec.length(), "\t", traction_force_z.length())

        w.position = s + Vector3.DOWN * distance
        if is_steering:
            w.rotation.y = deg_to_rad(-steering * max_steering_angle)
    # Air drag
    var air_drag_coef = 5
    var velo_sqr = linear_velocity.length_squared()
    if velo_sqr > 0.1:
        var air_drag = -linear_velocity.normalized() * velo_sqr * air_drag_coef
        apply_central_force(air_drag)


var accelerate = 0.
var braking = 0.
var steering = 0.
var steering_target = 0.
@export var steering_speed = 4

var wheel_torque_damp: float = 1.
var rpm: float = 0.

@export var max_steering_angle = 45.

func _input(event):
    if event.is_action("car_accelerate"):
        var input_acc = event.get_action_strength("car_accelerate")
        accelerate = clampf(input_acc, 0, 1)
    if event.is_action("car_brake"):
        var input_brk = event.get_action_strength("car_brake")
        braking = clampf(input_brk, 0, 1)
    if event.is_action("car_steer_left") or event.is_action("car_steer_right"):
        var input_left = event.get_action_strength("car_steer_left")
        var input_right = event.get_action_strength("car_steer_right")
        steering_target = clampf(input_right - input_left, -1, 1)
    #var input_steer = event.get_action_strength("car_steer")

func _process(delta):
    if accelerate > 0:
        rpm = clampf(rpm + accelerate * acceleration * delta, 0, max_rpm)
    if braking > 0:
        rpm = clampf(rpm - brake_force * braking * delta, 0, max_rpm)

    steering = move_toward(steering, steering_target, delta * steering_speed)
    rpm = move_toward(rpm, 0, wheel_torque_damp * delta)

func get_point_velocity (point :Vector3)->Vector3:
    return linear_velocity + angular_velocity.cross(point - to_global(center_of_mass))

func get_point_inertia(point: Vector3) -> Vector3:
    return inertia.cross(point - to_global(center_of_mass))
