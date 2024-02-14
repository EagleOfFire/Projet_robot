"""my_controller controller."""

from controller import Robot, Motor, DistanceSensor, RangeFinder, GPS
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
        self.range_finder: RangeFinder = self.getDevice('range-finder')

        self.position: GPS = self.getDevice('gps')

        self.right_wheel_1.setPosition(float('inf'))
        self.right_wheel_2.setPosition(float('inf'))
        self.right_wheel_3.setPosition(float('inf'))
        self.right_wheel_4.setPosition(float('inf'))
        self.left_wheel_1.setPosition(float('inf'))
        self.left_wheel_2.setPosition(float('inf'))
        self.left_wheel_3.setPosition(float('inf'))
        self.left_wheel_4.setPosition(float('inf'))

    def init_capteur(self, temps):
        self.range_finder.enable(temps)
        self.position.enable(temps)

    def set_vitesse(self, speed):
        self.right_wheel_1.setVelocity(speed)
        self.right_wheel_2.setVelocity(speed)
        self.right_wheel_3.setVelocity(speed)
        self.right_wheel_4.setVelocity(speed)

        self.left_wheel_1.setVelocity(speed)
        self.left_wheel_2.setVelocity(speed)
        self.left_wheel_3.setVelocity(speed)
        self.left_wheel_4.setVelocity(speed)

    def get_distance(self, image):
        width = int(self.range_finder.getWidth())
        x = int(self.range_finder.getWidth()*(3/4))
        y = int(self.range_finder.getHeight()*(3/4))
        Rdistance = self.range_finder.rangeImageGetDepth(image, width, x, y)

        width = int(self.range_finder.getWidth())
        x = int(self.range_finder.getWidth()/4)
        y = int(self.range_finder.getHeight()*(3/4))
        Ldistance = self.range_finder.rangeImageGetDepth(image, width, x, y)
        return Rdistance, Ldistance

    def turn_right(self):
        self.right_wheel_1.setVelocity(-26)
        self.right_wheel_2.setVelocity(-26)
        self.right_wheel_3.setVelocity(-26)
        self.right_wheel_4.setVelocity(-26)
        print("turning right")

    def turn_left(self):
        self.left_wheel_1.setVelocity(-26)
        self.left_wheel_2.setVelocity(-26)
        self.left_wheel_3.setVelocity(-26)
        self.left_wheel_4.setVelocity(-26)
        print("turning right")


robot = SPEEEED()
timestep = int(robot.getBasicTimeStep())
robot.init_capteur(timestep)
right_distance_precedent = 0
left_distance_precedent = 0
vitesse = 10

while robot.step(timestep) != -1:
    front_image = robot.range_finder.getRangeImage()
    right_distance, left_distance = robot.get_distance(front_image)
    positionX, positionY, positionZ = robot.position.getValues()
    print(f"positionX : {round(positionX)} , positionY : {round(positionY)}")

    if round(positionX) == -12 and round(positionY) == 15:
        right_distance = float('inf')
        left_distance = float('inf')
        vitesse = 10

    if right_distance > 2 and left_distance > 2:
        vitesse = 15
    else:
        vitesse = 10
    print(vitesse)

    if right_distance < 1 or right_distance < left_distance:
        robot.turn_left()
    elif left_distance < 1 or left_distance < right_distance:
        robot.turn_right()
    else:
        robot.set_vitesse(vitesse)
    right_distance_precedent = right_distance
    left_distance_precedent = left_distance
    pass
