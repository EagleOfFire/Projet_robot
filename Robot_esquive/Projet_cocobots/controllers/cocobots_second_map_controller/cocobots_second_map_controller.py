from controller import Robot, Motor, GPS, DistanceSensor, Compass
import math

# target coordinates
TARGET_X = -44.85
TARGET_Y = 50.97
TARGET_Z = 0.09


def angle_between_vectors(a, b):
    dot_product = sum(x * y for x, y in zip(a, b))
    norm_a = math.sqrt(sum(x * x for x in a))
    norm_b = math.sqrt(sum(x * x for x in b))
    return math.acos(dot_product / (norm_a * norm_b))
def calculate_opposite_angle(self, target_direction):
    robot_direction = self.compass.getValues()
    angle = math.atan2(-target_direction[1], -target_direction[0]) - math.atan2(robot_direction[1],
                                                                                robot_direction[0])
    return angle

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


class MyRobot(Robot):
    # maximal speed allowed
    __MAX_SPEED = 12.3

    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Get handles for motors
        self.left_wheel: Motor = self.getDevice('left wheel')
        self.right_wheel: Motor = self.getDevice('right wheel')

        # Set the maximum motor speed
        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))

        # Disable the motors
        self.left_wheel.setVelocity(0)
        self.right_wheel.setVelocity(0)

        # Get handle for GPS
        self.gps: GPS = self.getDevice('gps')
        self.gps.enable(self.timestep)

        # Get handle distance sensor
        self.distance_sensor_left_left_left: DistanceSensor = self.getDevice('so0')
        self.distance_sensor_left_left: DistanceSensor = self.getDevice('so1')
        self.distance_sensor_left: DistanceSensor = self.getDevice('so2')
        self.distance_sensor_center_left: DistanceSensor = self.getDevice('so3')
        self.distance_sensor_center_right: DistanceSensor = self.getDevice('so4')
        self.distance_sensor_right: DistanceSensor = self.getDevice('so5')
        self.distance_sensor_right_right: DistanceSensor = self.getDevice('so6')
        self.distance_sensor_right_right_right: DistanceSensor = self.getDevice('so7')

        self.all_distance_sensor_left = [self.distance_sensor_left_left_left, self.distance_sensor_left_left,
                                         self.distance_sensor_left, self.distance_sensor_center_left]
        self.all_distance_sensor_right = [self.distance_sensor_right_right_right, self.distance_sensor_right_right,
                                          self.distance_sensor_right, self.distance_sensor_center_right]

        self.compass: Compass = self.getDevice('compass')
        self.compass.enable(self.timestep)

        for sensor in self.all_distance_sensor_left:
            sensor.enable(self.timestep)
        for sensor in self.all_distance_sensor_right:
            sensor.enable(self.timestep)

    def set_speed(self, right_speed, left_speed):

        # Right wheel
        if right_speed > self.__MAX_SPEED:
            self.right_wheel.setVelocity(self.__MAX_SPEED)
        elif right_speed < -self.__MAX_SPEED:
            self.right_wheel.setVelocity(-self.__MAX_SPEED)
        else:
            self.right_wheel.setVelocity(right_speed)

        # Left wheel
        if left_speed > self.__MAX_SPEED:
            self.left_wheel.setVelocity(self.__MAX_SPEED)
        elif left_speed < -self.__MAX_SPEED:
            self.left_wheel.setVelocity(-self.__MAX_SPEED)
        else:
            self.left_wheel.setVelocity(left_speed)

    def avoid_collision(self):
        sum_distance_left = 0
        sum_distance_right = 0
        for i, sensor in enumerate(self.all_distance_sensor_left):
            sum_distance_left += (i * 10.0) * sensor.getValue()/self.distance_sensor_left.getMaxValue()  # i * sensor.getValue() / 100.0

        for i, sensor in enumerate(self.all_distance_sensor_right):
            sum_distance_right += (i * 10.0) * sensor.getValue()/self.distance_sensor_right.getMaxValue()  # (len(self.all_distance_sensor_right) - i) * sensor.getValue()/100.0

        self.set_speed(self.__MAX_SPEED - sum_distance_left, self.__MAX_SPEED - sum_distance_right)

        # self.set_speed(self.__MAX_SPEED - map_range(sum_distance_left, 0, self.distance_sensor_left.getMaxValue(),
        #                                             -self.__MAX_SPEED, self.__MAX_SPEED),
        #                self.__MAX_SPEED - map_range(sum_distance_right, 0, self.distance_sensor_right.getMaxValue(),
        #                                             -self.__MAX_SPEED, self.__MAX_SPEED))

        # print('dist : ', self.distance_sensor_right.getValue(), self.distance_sensor_left.getValue())
        # print('speed : ', self.right_wheel.getVelocity(), self.left_wheel.getVelocity())

    def calculate_opposite_angle(self, target_direction):
        robot_direction = self.compass.getValues()
        angle = math.atan2(-target_direction[1], -target_direction[0]) - math.atan2(robot_direction[1],
                                                                                    robot_direction[0])
        return angle

    def follow_direction(self, target_direction, angle_tolerance=0.1):
        opposite_direction = [-target_direction[0], -target_direction[1]]

        opposite_angle = self.calculate_opposite_angle(target_direction)

        if abs(opposite_angle) < angle_tolerance:
            self.left_wheel.setVelocity(self.__MAX_SPEED)
            self.right_wheel.setVelocity(self.__MAX_SPEED)
        else:
            if opposite_angle > 0:
                self.left_wheel.setVelocity(self.__MAX_SPEED)
                self.right_wheel.setVelocity(-self.__MAX_SPEED)
            else:
                self.left_wheel.setVelocity(-self.__MAX_SPEED)
                self.right_wheel.setVelocity(self.__MAX_SPEED)

    def run(self):
        target_direction = [-0.6848047797078073, 0.7285135176177693]
        while self.step(self.timestep) != -1:
            # if (self.all_distance_sensor_left[2].getValue() > 800 or self.all_distance_sensor_right[2].getValue() > 800):
            self.avoid_collision()
            #  else:
            self.follow_direction(target_direction)


# Create the Robot instance.
robot = MyRobot()

# Run the controller
robot.run()
