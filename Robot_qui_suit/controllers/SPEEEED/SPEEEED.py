"""my_controller controller."""

from controller import Robot, Camera, Gyro, Motor, DistanceSensor
from time import sleep


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


class SPEEEED(Robot):
    def __init__(self):
        super().__init__()
        self.right_wheel_1: Motor = self.getDevice('right_front_wheel')
        self.right_wheel_2: Motor = self.getDevice('right_rear_wheel')
        self.left_wheel_1: Motor = self.getDevice('left_front_wheel')
        self.left_wheel_2: Motor = self.getDevice('left_rear_wheel')

        self.left_steer: Motor = self.getDevice('left_steer')
        self.right_steer: Motor = self.getDevice('right_steer')

        self.side_left_sensor: DistanceSensor = self.getDevice('side_left_sensor')
        self.front_left_sensor: DistanceSensor = self.getDevice('front_left_sensor')
        self.front_center_sensor: DistanceSensor = self.getDevice('front_center_sensor')
        self.front_right_sensor: DistanceSensor = self.getDevice('front_right_sensor')
        self.side_right_sensor: DistanceSensor = self.getDevice('side_right_sensor')

        self.right_wheel_1.setPosition(float('inf'))
        self.right_wheel_2.setPosition(float('inf'))
        self.left_wheel_1.setPosition(float('inf'))
        self.left_wheel_2.setPosition(float('inf'))

    def robot_set_speed(self, speed_right, speed_left):
        self.right_wheel_1.setVelocity(speed_right)
        self.right_wheel_2.setVelocity(speed_right)
        self.left_wheel_1.setVelocity(speed_left)
        self.left_wheel_2.setVelocity(speed_left)

    def init_capteur(self, temps):
        self.side_left_sensor.enable(temps)
        self.front_left_sensor.enable(temps)
        self.front_center_sensor.enable(temps)
        self.front_right_sensor.enable(temps)
        self.side_right_sensor.enable(temps)

    @staticmethod
    def get_distance():
        side_leftD = robot.side_left_sensor.getValue()
        leftD = robot.front_left_sensor.getValue()
        centerD = robot.front_center_sensor.getValue()
        rightD = robot.front_right_sensor.getValue()
        side_rightD = robot.side_right_sensor.getValue()
        return leftD, centerD, rightD, side_leftD, side_rightD

    def go_ahead(self):
        self.left_steer.setPosition(0)
        self.right_steer.setPosition(0)

    def turn_right(self, coef):
        self.left_steer.setPosition(coef)
        self.right_steer.setPosition(coef)

    def turn_left(self, coef):
        self.left_steer.setPosition(coef)
        self.right_steer.setPosition(coef)


robot = SPEEEED()
timestep = int(robot.getBasicTimeStep())
robot.init_capteur(timestep)

while robot.step(timestep) != -1:
    robot.robot_set_speed(29.7, 29.7)
    left, center, right, side_left, side_right = robot.get_distance()
    robot.go_ahead()
    if left > 0 or side_left > 0:
        robot.turn_right(map_range(side_left, 0, 900, 0, 1.34))
    elif right > 0 or side_right > 0:
        robot.turn_right(map_range(side_right, 0, 900, -1.34, 0))
    print(f"left : {left} center : {center} right : {right} side_left : {side_left} side_right : {side_right}")
    pass
