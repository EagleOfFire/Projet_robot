"""my_controller controller."""

from controller import Robot, Camera, Gyro, Motor, DistanceSensor, RangeFinder
from time import sleep


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


class SPEEEED(Robot):
    def __init__(self):
        super().__init__()
        self.right_wheel_1: Motor = self.getDevice('right motor 1')
        self.right_wheel_2: Motor = self.getDevice('right motor 2')
        self.right_wheel_3: Motor = self.getDevice('right motor 3')
        self.right_wheel_4: Motor = self.getDevice('right motor 4')
        self.left_wheel_1: Motor = self.getDevice('left motor 1')
        self.left_wheel_2: Motor = self.getDevice('left motor 2')
        self.left_wheel_3: Motor = self.getDevice('left motor 3')
        self.left_wheel_4: Motor = self.getDevice('left motor 4')
        self.right_sensor: RangeFinder = self.getDevice('range-finder-right')
        self.left_sensor: RangeFinder = self.getDevice('range-finder-left')

        self.right_wheel_1.setPosition(float('inf'))
        self.right_wheel_2.setPosition(float('inf'))
        self.right_wheel_3.setPosition(float('inf'))
        self.right_wheel_4.setPosition(float('inf'))
        self.left_wheel_1.setPosition(float('inf'))
        self.left_wheel_2.setPosition(float('inf'))
        self.left_wheel_3.setPosition(float('inf'))
        self.left_wheel_4.setPosition(float('inf'))

    def init_capteur(self, temps):
        self.left_sensor.enable(temps)
        self.right_sensor.enable(temps)

    def set_vitesse(self, speed):
        self.right_wheel_1.setVelocity(speed)
        self.right_wheel_2.setVelocity(speed)
        self.right_wheel_3.setVelocity(speed)
        self.right_wheel_4.setVelocity(speed)

        self.left_wheel_1.setVelocity(speed)
        self.left_wheel_2.setVelocity(speed)
        self.left_wheel_3.setVelocity(speed)
        self.left_wheel_4.setVelocity(speed)

    def get_distance_left(self, image):
        width = int(self.left_sensor.getWidth())
        x = int(self.left_sensor.getWidth()/2)
        y = int(self.left_sensor.getHeight()/2)
        return self.left_sensor.rangeImageGetDepth(image, width, x, y)

    def get_distance_right(self, image):
        width = int(self.right_sensor.getWidth())
        x = int(self.right_sensor.getWidth()/2)
        y = int(self.right_sensor.getHeight()/2)
        return self.right_sensor.rangeImageGetDepth(image, width, x, y)


robot = SPEEEED()
timestep = int(robot.getBasicTimeStep())
robot.init_capteur(timestep)
robot.set_vitesse(26)

while robot.step(timestep) != -1:
    left_image = robot.left_sensor.getRangeImage()
    right_image = robot.right_sensor.getRangeImage()
    left_distance = robot.get_distance_left(left_image)
    right_distance = robot.get_distance_right(right_image)
    print(f"distance left : {left_distance} distance right : {right_distance}")
    pass
