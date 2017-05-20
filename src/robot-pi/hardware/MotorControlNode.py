#!/usr/bin/env python

import rospy as rp
from turmc.global_constants import *
from turmc.tools.textutils import unpackDict
from turmc.motor_control.pwm import MicrostepStepperMotor
from turmc.motor_control.sabertooth_serial import DCMotor, LinearActuator, DrivetrainSimplified

#Processes Strings from the subscribed topic
def callback(data):
    #Converts command dictionary from JSON
    commands = unpackDict(data.data)

    for command in commands:
        if command == 'drivetrain':
            drivetrain.drive(commands[command])
        elif command == 'drillActuator':
            if commands[command] == 'extend':
                drillActuator.extend()
            elif commands[command] == 'retract':
                drillActuator.retract()
            else:
                drillActuator.stop()
        elif command == 'conveyorActuator':
            if commands[command] == 'extend':
                conveyorActuator.extend()
            elif commands[command] == 'retract':
                conveyorActuator.retract()
            else:
                conveyorActuator.stop()
        elif command == 'auger':
            if commands[command] == 'start':
                auger.setSpeed(1.0)
            elif commands[command] == 'reverse':
                auger.setSpeed(-0.5)
            else:
                auger.stop()
        elif command == 'conveyorMotor':
            if commands[command] == 'start':
                conveyorMotor.setSpeed(0.5)
            elif commands[command] == 'reverse':
                conveyorMotor.setSpeed(-0.2)
            else:
                conveyorMotor.stop()
        elif command == 'drillStepper':
            if commands[command] == 'up':
                drillStepper.up()
                drillStepper.setPace(5)
            elif commands[command] == 'down':
                drillStepper.down()
                drillStepper.setPace(5)
            else:
                drillStepper.hold()

def init():
    global drivetrain, drillActuator, conveyorActuator, auger, conveyorMotor, drillStepper

    #Define all the motor objects
    drivetrain = DrivetrainSimplified(SERIAL_ADDR_DRIVETRAIN, invertY = True)
    drillActuator = LinearActuator(SERIAL_ADDR_LINEAR_ACUATORS, MOTOR_NUMBER_DRILL_ACTUATOR)
    conveyorActuator = LinearActuator(SERIAL_ADDR_LINEAR_ACUATORS, MOTOR_NUMBER_CONVEYOR_ACTUATOR)
    auger = DCMotor(SERIAL_ADDR_OTHER_MOTORS, MOTOR_NUMBER_AUGER)
    conveyorMotor = DCMotor(SERIAL_ADDR_OTHER_MOTORS, MOTOR_NUMBER_CONVEYOR, invertCommands = True) #DO NOT EXCEED SPEED OF 0.5 FOR THIS MOTOR
    drillStepper = MicrostepStepperMotor(DRILL_STEPPER_PUL_PIN, DRILL_STEPPER_DIR_PIN)

    #Initialize the node
    rp.init_node('MotorControlNode', anonymous = True)

    #Configures shutdown hook
    rp.on_shutdown(shutdown)

    #Initialize the subscriber
    subscriber = rp.Subscriber(TOPIC_MOTOR_CONTROL, String, callback)

    return

def shutdown():
    drivetrain.drive(0.0, 0.0)
    drillActuator.stop()
    conveyorActuator.stop()
    auger.stop()
    conveyorMotor.stop()
    drillStepper.hold()
    return

def main():

    init()

    rp.spin()

    shutdown()

    return

if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        rp.loginfo('MotorControlNode interrupted; shutting down')
        shutdown()
        pass
