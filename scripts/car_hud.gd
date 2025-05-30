extends Control

@export var car: Car

@export var speedLabel: Label
@export var rpmLabel: Label
@export var gearLabel: Label

func _process(delta):
    speedLabel.text = str(abs(roundi(car.current_speed * 3.600))) + " km/h"
    rpmLabel.text = str(floori(car.current_rpm)) + " rpm"
    match(car.gearbox_state):
        Car.GEARBOX_STATE.GEAR:
            gearLabel.text = str(car.current_gear)
        Car.GEARBOX_STATE.REVERSE:
            gearLabel.text = "R"
        Car.GEARBOX_STATE.SHIFTING or Car.GEARBOX_STATE.NEUTRAL:
            gearLabel.text = "N"
