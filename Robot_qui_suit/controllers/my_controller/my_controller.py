"""my_controller controller."""

from controller import Robot, Camera, Gyro, Motor
from time import sleep


class BB_8_controller(Robot):
    def __init__(self):
        super().__init__()
        self.camera: Camera = self.getDevice('camera')
        self.counterweight_gyro: Gyro = self.getDevice('counterweight gyro')
        self.body_pitch_motor: Motor = self.getDevice('body pitch motor')
        self.body_yaw_motor: Motor = self.getDevice('body yaw motor')
        self.head_yaw_motor: Motor = self.getDevice('head yaw motor')


robot = BB_8_controller()
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    robot.camera.enable(p=timestep)
    robot.counterweight_gyro.enable(p=timestep)
    robot.body_pitch_motor.setVelocity(0)
    robot.body_yaw_motor.setVelocity(0)
    robot.head_yaw_motor.setVelocity(0)

    if robot.camera.hasRecognition():
        robot.camera.recognitionEnable(robot.camera.getSamplingPeriod())
        print("Recognition ON")

    counterweight_x_vector, counterweight_y_vector, counterweight_z_vector = robot.counterweight_gyro.getValues()
    print(f"angle X : {counterweight_x_vector} angle Y : {counterweight_y_vector} angle Z : {counterweight_z_vector}")
    pass

