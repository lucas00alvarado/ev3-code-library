from ev3dev2.motor import *
from ev3dev2.sensor.lego import *


class Robot(MoveTank):
    def __init__(self, left_motor_port, right_motor_port, left_sensor_port=None, right_sensor_port=None,
                 back_sensor_port=None, gyro_sensor_port=None, motor1_port=None, motor2_port=None):
        """
A class that contains all of the functions that the ev3 should need to use. It has functionality for line followers,
gyro sensor programs, driving, and more. Also contains all of the sensor and motor objects so that all of it only needs
to be initialized once when the Robot class is initialized. Some of the methods inside of this class will only work if
you have passed in the ports for all of the sensors and motors.
        :param left_motor_port: port for the left driving motor. Values: OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
        :param right_motor_port: port for the right driving motor. Values: OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
        :param left_sensor_port: port for the left color sensor. Values: INPUT_1, INPUT_2, INPUT_3, INPUT_4
        :param right_sensor_port: port for the right color sensor. Values: INPUT_1, INPUT_2, INPUT_3, INPUT_4
        :param back_sensor_port: port for the back color sensor. Values: INPUT_1, INPUT_2, INPUT_3, INPUT_4
        :param gyro_sensor_port: port for the gyro sensor. Values: INPUT_1, INPUT_2, INPUT_3, INPUT_4
        :param motor1_port: port for one of the attachment motors. Values: OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
        :param motor2_port: port for one of the attachment motors. Values: OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
        """
        super().__init__(left_motor_port, right_motor_port)
        self.motor1 = motor1_port
        self.motor2 = motor2_port
        self.gyro_sensor = GyroSensor(gyro_sensor_port)
        self.back_sensor = ColorSensor(back_sensor_port)
        self.left_sensor = ColorSensor(left_sensor_port)
        self.right_sensor = ColorSensor(right_sensor_port)
        self.left_motor = LargeMotor(left_motor_port)
        self.right_motor = LargeMotor(right_motor_port)
        self.black_value = 10
        self.white_value = 60

    def pid_base_code(self, error, speed, kp, ki, kd, info):
        """
Provides the PID calculations that are then used in the line followers and gyro straight driving programs.
        :param error: calculated deviation from desired location
        :param speed: speed to drive at
        :param kp: sharpness of corrections
        :param ki: keeps driving in a straight line by keeping track of overall errors
        :param kd: decreases amount of swinging in the line follower or gyro straight driving program
        :param info: info needed for integral and derivative to function
        :return: Info with a few tweaks which will then be passed back in though info in the next loop iteration
        """
        proportional = error * kp
        integral = (error + info[0]) * ki
        derivative = (info[1] - error) * kd
        correction = proportional + integral + derivative
        self.on(speed - correction, speed + correction)  # Super Function
        return [error + info[0], error]

    def follow_until_black(self, color_sensor: ColorSensor, stop_sensor: ColorSensor, speed, rli, kp, ki=0, kd=0):
        """
Allows you to follow a line until the other color sensor hits black. You have to use negative PID values if you are line following on the left side of a line
        :param color_sensor: the color sensor object that you would like to follow the line
        :param stop_sensor: the color sensor object to detect black
        :param speed: speed for line following. Use a negative value to go backwards
        :param rli: the rli that you would like for the robot to try to stay on. Go to https://docs.google.com/document/d/1oJ3-Bsqdp4RnKgrdCS8U93gZ9PlmqChw0xg6aUa2zJY/edit?usp=sharing to learn more.
        :param kp: sharpness of corrections in the line follower.
        :param ki: makes sure that your corrections keep you on a straight line. DON'T USE WITH TURNS
        :param kd: keeps your turning from continuing to swing back and forth
        """
        info = [0, 0]
        while stop_sensor.reflected_light_intensity > 10:
            error = color_sensor.reflected_light_intensity - rli
            info = self.pid_base_code(error, speed, kp, ki, kd, info)
        self.stop()  # Super Function

    def follow_until_white(self, color_sensor: ColorSensor, stop_sensor: ColorSensor, speed, rli, kp, ki=0, kd=0):
        """
Allows you to follow a line until the other color sensor hits white. You have to use negative PID values if you are line following on the left side of a line
        :param color_sensor: the color sensor object that you would like to follow the line
        :param stop_sensor: the color sensor object to detect white
        :param speed: speed for line following. Use a negative value to go backwards
        :param rli: the rli that you would like for the robot to try to stay on. Go to https://docs.google.com/document/d/1oJ3-Bsqdp4RnKgrdCS8U93gZ9PlmqChw0xg6aUa2zJY/edit?usp=sharing to learn more.
        :param kp: sharpness of corrections in the line follower.
        :param ki: makes sure that your corrections keep you on a straight line. DON'T USE WITH TURNS
        :param kd: keeps your turning from continuing to swing back and forth
        """
        info = [0, 0]
        while stop_sensor.reflected_light_intensity < self.white_value:
            error = color_sensor.reflected_light_intensity - rli
            info = self.pid_base_code(error, speed, kp, ki, kd, info)
        self.stop()  # Super Function

    def single_follow_distance(self, color_sensor: ColorSensor, speed, distance, rli, kp, ki=0, kd=0):
        """
Allows you to follow a line with 1 color sensor for a distance. You have to use negative PID values if you are line following on the left side of a line
        :param color_sensor: the color sensor object that you would like to follow the line
        :param speed: speed for line following. Use a negative value to go backwards
        :param distance: distance to line follow for in rotations. Use a negative value to go backwards
        :param rli: the rli that you would like for the robot to try to stay on. Go to https://docs.google.com/document/d/1oJ3-Bsqdp4RnKgrdCS8U93gZ9PlmqChw0xg6aUa2zJY/edit?usp=sharing to learn more
        :param kp: sharpness of corrections in the line follower
        :param ki: makes sure that your corrections keep you on a straight line. DON'T USE WITH TURNS
        :param kd: keeps your turning from continuing to swing back and forth
        """
        self.left_motor.position = 0
        self.right_motor.position = 0
        info = [0, 0]
        if distance > 0:
            while (self.left_motor.position + self.right_motor.position) / 2 < distance * 360:
                # distance * 360(line above) converts rotations into tachocounts
                error = color_sensor.reflected_light_intensity - rli
                info = self.pid_base_code(error, speed, kp, ki, kd, info)
            self.stop()  # Super Function
        else:
            while (self.left_motor.position + self.right_motor.position) / 2 > distance * 360:
                # distance * 360(line above) converts rotations into tachocounts
                error = color_sensor.reflected_light_intensity - rli
                info = self.pid_base_code(error, speed, kp, ki, kd, info)
            self.stop()  # Super Function

    def double_follow_distance(self, speed, distance, kp, ki=0, kd=0):
        """
Allows you to follow a line with 2 color sensors for a distance
        :param speed: speed for line following
        :param distance: distance to line follow for in rotations
        :param kp: sharpness of corrections in the line follower
        :param ki: makes sure that your corrections keep you on a straight line. DON'T USE WITH TURNS
        :param kd: keeps your turning from continuing to swing back and forth
        """
        self.left_motor.position = 0
        self.right_motor.position = 0
        info = [0, 0]
        while (self.left_motor.position + self.right_motor.position) / 2 < distance * 360:
            error = self.right_sensor.reflected_light_intensity - self.left_sensor.reflected_light_intensity
            info = self.pid_base_code(error, speed, kp, ki, kd, info)

    def gyro_straight(self, speed, distance, kp, ki=0, kd=0, angle=0):
        """
Allows you to drive in a straight line using a gyro sensor. You have to use negative PID values if going backwards.
You should also reset your gyro sensor with the command robot.gyro_sensor.reset() unless you want to follow a previous
reset.
        :param speed: speed for driving. Use a negative value to go backwards
        :param distance: distance to drive for in rotations. Use a negative value to go backwards
        :param kp: sharpness of corrections in your driving
        :param ki: makes sure that your corrections keep you on a straight line
        :param kd: keeps your turning from continuing to swing back and forth
        :param angle: the gyro sensor angle that you want to follow the line at
        """
        self.left_motor.position = 0
        self.right_motor.position = 0
        info = [0, 0]
        while (self.left_motor.position + self.right_motor.position) / 2 < distance * 360:
            error = self.gyro_sensor.angle - angle
            info = self.pid_base_code(error, speed, kp, ki, kd, info)

    def gyro_turn(self, angle, left_speed, right_speed, buffer=2):
        """
Allows you to turn a specific angle using the gyro sensor. You should reset your gyro sensor with the command
robot.gyro_sensor.reset() unless you want to turn using a previous reset
        :param angle: gyro angle to turn to. Use a negative value if turning counter-clockwise
        :param left_speed: the speed that the left wheel should drive at during the turn
        :param right_speed: the speed that the right wheel should drive at during the turn
        :param buffer: the amount of buffer in degrees that it can be on either side of the angle
        """
        while not ((angle + buffer) > self.gyro_sensor.angle > (angle - buffer)):
            self.on(left_speed, right_speed)  # Super Function
        self.stop()

    def square_line(self, speed):
        """
Allows the robot to square up on a black line so that the color sensors are parallel to the line
        :param speed: the speed to drive at during the square up
        """
        while self.left_sensor.color_name != "Black" and self.right_sensor.color_name != "Black":
            self.on(speed, speed)
        self.stop()
        while self.left_sensor.color_name != "Black":
            self.on(speed, 0)  # Super Function
        while self.right_sensor.color_name != "Black":
            self.on(0, speed)  # Super Function

    def stop_on_color(self, color_name, left_speed, right_speed, single_sensor=False, sensor=None):
        """
Allows you to have the robot drive and then stop on a certain color
        :param color_name: the color to stop on. Values: No color, Black, Blue, Green, Yellow, Red, White, and Brown
        :param left_speed: the speed that the left wheel should drive at
        :param right_speed: the speed that the right wheel should drive at
        :param single_sensor: whether it should be waiting for a specific sensor or for one of the front two
        :param sensor: the sensor to wait for if single_sensor is True
        """
        if single_sensor:
            while sensor.color_name != color_name:
                self.on(left_speed, right_speed)  # Super Function
        else:
            while self.left_sensor.color_name != color_name and self.right_sensor.color_name != color_name:
                self.on(left_speed, right_speed)  # Super Function


