"""cocoRobotController controller."""

from controller import Robot
import numpy as np


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 69.0  # with this thrust, the drone lifts.
    K_VERTICAL_P = 2.5       # P constant of the vertical PID.
    K_VERTICAL_D = 200.0  # P constant of the vertical PID.
    K_ROLL_P = 50.0        # P constant of the roll PID.
    K_PITCH_P = 50.0        # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.1
    travel_altitude = 1

    dictPipes = {1: [[-74, 67, 2], [-74, 84, 2]],
                 2: [[-60.7, 65.5, 7], [-60.7, 73.5, 7]],
                 3: [[-50, 72, 4], [-57.63, 80.45, 4]],
                 4: [[-65, 59.68, 10], [-72, 84, 10]]}

    depart=[-47,57,0.1]
    # rota= 1.552 altitude = 0.1

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

        self.flag = 1


    def update_flag(self):
        self.flag += 1
        if self.flag > 4:
            self.flag = 1

    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos

    def goToTravelAltitude(self):
        self.target_altitude = self.target_position[2]

    def goToTargetAltitude(self):
        self.target_altitude = self.target_position[2]

    def move_to_target(self, waypoints):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """
        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            self.update_flag()


        # This will be in ]-pi;pi]
        position = np.arctan2(self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = position - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)

        distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + ((self.target_position[1] - self.current_pose[1]) ** 2))
        # non proportional and decreasing function
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, -self.MAX_PITCH_DISTURBANCE)

        return yaw_disturbance, pitch_disturbance

    def run(self):
        last_clamped_difference_altitude = 0
        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        # Specify the patrol coordinates
        # waypoints = self.dictPipes.get(3).append(self.dictPipes.get(1)).append(self.dictPipes.get(4)).append(self.dictPipes.get(2))
        waypoints = [[-50, 72, 4], [-57.63, 80.45, 4], [-74, 84, 2],[-74, 67, 2], [-75, 59.8, 10],[-65, 59.68, 10],[-60.7, 65.5, 7], [-60.7, 73.5, 7]]


        self.target_position = waypoints[0] # init

        while self.step(self.time_step) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            if self.flag == 1:
                self.goToTravelAltitude()
                if self.target_altitude - self.current_pose[2] <= 0.1:
                    self.update_flag()

            if self.flag == 2:
                self.target_precision = 0.1
                yaw_disturbance, pitch_disturbance = self.move_to_target(waypoints)
            if self.flag == 4:
                self.target_precision = 1
                yaw_disturbance, pitch_disturbance = self.move_to_target(waypoints)

            if self.flag == 3:
                self.goToTargetAltitude()
                if self.target_altitude - self.current_pose[2] <= 0.1:
                    self.update_flag()

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance

            # PID for control altitude
            clamped_difference_altitude = clamp(self.target_altitude - altitude, -1, 1)
            d_clamped_difference_altitude = clamped_difference_altitude - last_clamped_difference_altitude
            last_clamped_difference_altitude = clamped_difference_altitude
            vertical_input = self.K_VERTICAL_P * clamped_difference_altitude + self.K_VERTICAL_D * d_clamped_difference_altitude

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()
