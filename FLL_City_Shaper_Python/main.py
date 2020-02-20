#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

from threading import Thread

# Write your program here
brick.sound.beep()


class Robot:

    # wheel diameter in mm
    WHEEL_DIAMETER = 64.2
    # distance between wheels in mm
    AXLE_TRACK = 200

    BACKLASH_DUTY = 15

    motor_left = Motor(Port.C,Direction.COUNTERCLOCKWISE,[12,20])
    motor_right = Motor(Port.B,Direction.CLOCKWISE,[12,20])

    light_sensor_right = ColorSensor(Port.S1)
    light_sensor_left = ColorSensor(Port.S2)

    gyro_sensor = GyroSensor(Port.S3)
    ultrasonic_sensor = UltrasonicSensor(Port.S4)

    drive_base = DriveBase(motor_left, motor_right, WHEEL_DIAMETER,AXLE_TRACK)




    def p_line_Follow(self,_sensor,_sensor_target,_sensor_gain,_base_speed):

        # target light sensor value
        light_sensor_target = _sensor_target
        # light sensor gain
        light_sensor_p_gain = _sensor_gain
        # desired base speed of the drivetrain
        base_speed = _base_speed
        
        while(True):    

            Kp = light_sensor_p_gain*( light_sensor_target - _sensor.reflection() )

            self.motor_left.run(base_speed - Kp)
            self.motor_right.run(base_speed + Kp)


    


    def move_straight(self,_speed,_degrees,_stop_type):
     
        # thread that checks for stall condition
        def check_stall():

            while(True):
                if self.motor_left.stalled() or self.motor_right.stalled():
                    self.motor_left.stop(Stop.HOLD)
                    self.motor_right.stop(Stop.HOLD)
                    brick.sound.beep()
                    break
      
        # motor backlash compensation
        def backlash_compensation(self,_degrees):
            if(_degrees > 0 ):
                self.motor_left.dc(self.BACKLASH_DUTY)
                self.motor_right.dc(self.BACKLASH_DUTY)        
            elif(_degrees < 0):
                self.motor_left.dc(self.BACKLASH_DUTY*-1)
                self.motor_right.dc(self.BACKLASH_DUTY*-1)     
            else:
                return        
            # duration to remove lash from motors
            wait(100)
            # reset rotation sensors
            self.motor_left.reset_angle(0)
            self.motor_right.reset_angle(0)

        # compensate for the backlash
        backlash_compensation(_degrees)

        # start thread to detect stall condition
        t_check_stall = Thread(target=check_stall, args=())
        t_check_stall.start()

        self.motor_left.run_target(_speed, _degrees, _stop_type, False)
        self.motor_right.run_target(_speed, _degrees, _stop_type, True)

        # turn off stall detection thread
        t_check_stall.exit()



# initialize robot
robot = Robot()

robot.move_straight(200,720,Stop.HOLD)

# robot.p_line_Follow(robot.light_sensor_right,50,1,200)

