from controller import Robot, Motor, GPS, DistanceSensor, Gyro
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

        # Get handle for gyroscope
        self.gyro: Gyro = self.getDevice('gyro')
        self.gyro.enable(self.timestep)

        # Get handle distance sensor
        self.distance_sensor_left_left: DistanceSensor = self.getDevice('so1')
        self.distance_sensor_left: DistanceSensor = self.getDevice('so2')
        self.distance_sensor_center_left: DistanceSensor = self.getDevice('so3')
        self.distance_sensor_center_right: DistanceSensor = self.getDevice('so4')
        self.distance_sensor_right: DistanceSensor = self.getDevice('so5')
        self.distance_sensor_right_right: DistanceSensor = self.getDevice('so6')
        self.distance_sensor_left_left.enable(self.timestep)
        self.distance_sensor_left.enable(self.timestep)
        self.distance_sensor_center_left.enable(self.timestep)
        self.distance_sensor_center_right.enable(self.timestep)
        self.distance_sensor_right.enable(self.timestep)
        self.distance_sensor_right_right.enable(self.timestep)

        self.all_distance_sensor_left = [self.distance_sensor_left_left,
                                         self.distance_sensor_left, self.distance_sensor_center_left]
        self.all_distance_sensor_right = [self.distance_sensor_right_right,
                                         self.distance_sensor_right, self.distance_sensor_center_right]

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

        for sensor in self.all_distance_sensor_left:
            sum_distance_left += sensor.getValue()

        for sensor in self.all_distance_sensor_right:
            sum_distance_right += sensor.getValue()

        self.set_speed(self.__MAX_SPEED - map_range(sum_distance_left, 0, self.distance_sensor_left.getMaxValue(),
                                                    -self.__MAX_SPEED, self.__MAX_SPEED),
                       self.__MAX_SPEED - map_range(sum_distance_right, 0, self.distance_sensor_right.getMaxValue(),
                                                    -self.__MAX_SPEED, self.__MAX_SPEED))
        # print('dist : ', self.distance_sensor_right.getValue(), self.distance_sensor_left.getValue())
        # print('speed : ', self.right_wheel.getVelocity(), self.left_wheel.getVelocity())

    def compute_speed_vector(self):
        pass
    def run(self):
        while self.step(self.timestep) != -1:
            # Get current position from GPS
            current_position = self.gps.getValues()

            self.avoid_collision()
            # Compute difference between current position and target position
            dx = TARGET_X - current_position[0]
            dy = TARGET_Y - current_position[1]
            dz = TARGET_Z - current_position[2]

            # Calculate the angle to the target
            # angle_to_target = -atan2(dy, dx)
            angle_to_target = angle_between_vectors([dx, dy], [current_position[0] - (current_position[0] + 3),
                                                               current_position[1]])

            # Get current angular velocity from the gyroscope
            gyro_rate_y = self.gyro.getValues()[1]  # Assuming gyro returns values for Y-axis

            # Integrate gyro rate to get the orientation angle (in radians)
            current_orientation = angle_to_target + gyro_rate_y * self.timestep / 1000.0  # Convert timestep to seconds

            # Calculate the difference between the current orientation and the angle to the target
            delta_angle = angle_to_target - current_orientation

            # print(current_orientation, angle_to_target)


            # # Adjust motor velocities based on the difference
            # if abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dz) > 0.1:  # Threshold for stopping
            #
            #     self.set_speed(self.right_wheel.getVelocity() - delta_angle,
            #                    self.left_wheel.getVelocity() + delta_angle)
            #
            # else:
            #     # Stop the robot when it reaches the target with a 3 degree error margin
            #     if abs(delta_angle) < 3 * (math.pi / 180):  # 3 degrees in radians
            #         self.left_wheel.setVelocity(0)
            #         self.right_wheel.setVelocity(0)


# Create the Robot instance.
robot = MyRobot()

# Run the controller
robot.run()
