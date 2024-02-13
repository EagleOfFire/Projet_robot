"""cocoRobotController controller."""

from controller import Robot, Motor, Camera, GPS, Gyro, InertialUnit
import numpy as np
import matplotlib.pyplot as plt
import sys

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Motors():

    def __init__(self, robot):
        self.robot: Robot = robot
        self.front_left_motor: Motor = robot.getDevice("front left propeller")
        self.front_right_motor: Motor = robot.getDevice("front right propeller")
        self.rear_left_motor: Motor = robot.getDevice("rear left propeller")
        self.rear_right_motor: Motor = robot.getDevice("rear right propeller")
        self.motors = [self.front_left_motor, self.front_right_motor,
                       self.rear_left_motor, self.rear_right_motor]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

    def stop(self):
        for motor in self.motors:
            motor.setVelocity(0)

    def accelerate(self):
        for motor in self.motors:
            motor.setVelocity(motor.getVelocity()+1)

    # def turn_left(self):
    #
    # def goStraight(self):


class CocoRobotVolant(Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 69.12  # with this thrust, the drone lifts.
    K_VERTICAL_P = 2.67  # P constant of the vertical PID.
    K_VERTICAL_D = 200  # P constant of the vertical PID.
    K_ROLL_P = 50.0  # P constant of the roll PID.
    K_PITCH_P = 30.0  # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.5

    def __init__(self):
        super().__init__()
        self.motors = Motors(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera: Camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu: InertialUnit = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps: GPS = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro: Gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.camera_pitch_motor = self.getDevice("camera pitch")
        # self.camera_pitch_motor.setPosition(0.7)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

        if self.camera.hasRecognition():
            self.camera.recognitionEnable(self.time_step)


    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos

    def move_to_target(self, waypoint):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """
        self.target_position[0:2] = waypoint

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])

        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]

        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)

        # non proportional and decreasing function
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        # distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + ((self.target_position[1] - self.current_pose[1]) ** 2))

        return yaw_disturbance, pitch_disturbance
    def run(self):
        t1 = self.getTime()
        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        last_clamped_difference_altitude = 0

        # target altitude of the robot in meters
        self.target_altitude = 5
        # Specify the patrol coordinates
        waypoint = [0, 0]
        i = 0

        objects_dict = []
        while self.step(self.time_step) != -1:

            # detect objects
            if self.camera.hasRecognition():
                objects = self.camera.getRecognitionObjects()
                for object in objects:
                    objects_dict.append({'position': object.getPosition(), 'size': object.getSize(),
                                         'id': object.getId(), 'colors': object.getColors()})

            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # if altitude > self.target_altitude - 1:
            #     # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
            #     if self.getTime() - t1 > 0.1:
            #         yaw_disturbance, pitch_disturbance = self.move_to_target(waypoint)
            #         t1 = self.getTime()

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance

            clamped_difference_altitude = clamp(self.target_altitude - altitude, -1, 1)
            d_clamped_difference_altitude = clamped_difference_altitude - last_clamped_difference_altitude
            last_clamped_difference_altitude = clamped_difference_altitude
            vertical_input = self.K_VERTICAL_P * clamped_difference_altitude + self.K_VERTICAL_D * d_clamped_difference_altitude

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.motors.front_left_motor.setVelocity(front_left_motor_input)
            self.motors.front_right_motor.setVelocity(-front_right_motor_input)
            self.motors.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.motors.rear_right_motor.setVelocity(rear_right_motor_input)

            print(self.camera.getRecognitionNumberOfObjects(), objects_dict)

            # if i == 100:
            #     plt.figure()
            #     plt.imshow(self.camera.getRecognitionSegmentationImage())
            #     plt.show()
            #
            # i += 1


# create the Robot instance.
cocoRobot = CocoRobotVolant()
cocoRobot.run()

# Enter here exit cleanup code.
print("Vol terminé")
