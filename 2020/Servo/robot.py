#!/usr/bin/env python3
"""
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
"""

import wpilib
#import our motor controllers
import rev
import ctre

from wpilib.drive import MecanumDrive
from wpilib.interfaces import GenericHID
Hand = GenericHID.Hand
# Enable Driver Station Support
from networktables import NetworkTables


class MyRobot(wpilib.TimedRobot):
    
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 3
    rearLeftChannel = 4
    frontRightChannel = 2
    rearRightChannel = 1
    leftLift = 13
    rightLift = 14
    leftArm = 0
    rightArm = 1

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        # Construct the Network Tables Object
        self.sd = NetworkTables.getTable('SmartDashboard')
        self.sd.putNumber('RobotSpeed', .5)
        self.leftArm = wpilib.Servo(self.leftArm)
        self.leftArm.setBounds(2.0, 1.8, 1.5, 1.2, 1.0)
        self.rightArm = wpilib.Servo(self.rightArm)
        self.rightArm.setBounds(2.0, 1.8, 1.5, 1.2, 1.0)
        

        self.stick = wpilib.XboxController(self.joystickChannel)

        self.robotSpeed= self.sd.getNumber('RobotSpeed', .5)


    def teleopInit(self):
        #self.drive.setSafetyEnabled(True)

        self.robotSpeed= self.sd.getNumber('RobotSpeed', .5)

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        leftArmAngle = self.leftArm.get()
        print(leftArmAngle)
        self.leftArm.set(-1*(self.stick.getRawAxis(1)))        
        self.rightArm.set(-1*(self.stick.getRawAxis(1)))


if __name__ == "__main__":
    wpilib.run(MyRobot)