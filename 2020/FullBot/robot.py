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
    joystickChannel2 = 1

    def robotInit(self):
        # Construct the Network Tables Object
        self.sd = NetworkTables.getTable('SmartDashboard')
        self.sd.putNumber('RobotSpeed', .5)
        #self.motor = rev.CANSparkMax(1, rev.MotorType.kBrushless)
        """Robot initialization function"""
        self.frontLeftMotor = rev.CANSparkMax(self.frontLeftChannel, rev.MotorType.kBrushless)
        self.frontLeftMotor.restoreFactoryDefaults()
        self.rearLeftMotor = rev.CANSparkMax(self.rearLeftChannel, rev.MotorType.kBrushless)
        self.rearLeftMotor.restoreFactoryDefaults()
        self.frontRightMotor = rev.CANSparkMax(self.frontRightChannel, rev.MotorType.kBrushless)
        self.frontRightMotor.restoreFactoryDefaults()
        self.rearRightMotor = rev.CANSparkMax(self.rearRightChannel, rev.MotorType.kBrushless)
        self.rearRightMotor.restoreFactoryDefaults()
        #Servo for the shooter angle

        #Lift
        self.leftLift = rev.CANSparkMax(self.leftLift, rev.MotorType.kBrushless)
        self.leftLift.IdleMode(rev.IdleMode.kBrake)
            #lift 1 is the motor that moves the hook up.
        self.rightLift = rev.CANSparkMax(self.rightLift, rev.MotorType.kBrushless)
        #self.rightLift.IdleMode(rev.IdleMode.kBrake)
        self.rightLift.setInverted(True)
        self.rightLift.follow(self.leftLift,False)
    

        self.shooter = rev.CANSparkMax(5, rev.MotorType.kBrushless)
        self.intake = ctre.WPI_VictorSPX(7)

        #intake & shooter

        # invert the left side motors
        self.frontLeftMotor.setInverted(True)

        # you may need to change or remove this to match your robot
        self.rearLeftMotor.setInverted(True)

        self.rearRightMotor.setInverted(True)

        self.frontRightMotor.setInverted(True)

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )
        

        self.stick = wpilib.XboxController(self.joystickChannel)
        self.stick2 = wpilib.XboxController(self.joystickChannel2)

        self.robotSpeed= self.sd.getNumber('RobotSpeed', .5)
        #HoodArms
        self.leftArm = wpilib.Servo(self.leftArm)
        self.leftArm.setBounds(2.0, 1.8, 1.5, 1.2, 1.0)
        self.rightArm = wpilib.Servo(self.rightArm)
        self.rightArm.setBounds(2.0, 1.8, 1.5, 1.2, 1.0)

    def teleopInit(self):
        self.drive.setSafetyEnabled(True)

        self.robotSpeed= self.sd.getNumber('RobotSpeed', .5)
        self.liftSpeed = self.sd.getNumber('Lift Speed', .5)
    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # This sample does not use field-oriented drive, so the gyro input is set to zero.
        #self.motor.set(-0.47)
        # drive code...
       
        self.drive.driveCartesian((self.stick.getRawAxis(0)*self.robotSpeed), 
        
            (self.stick.getRawAxis(1)*self.robotSpeed), (self.stick.getRawAxis(4)*self.robotSpeed))
        self.intake.set(self.stick.getRawAxis(2)/2)
        self.shooter.set(self.stick.getRawAxis(3))
        self.leftLift.set(self.stick2.getRawAxis(1))
        #Shooter Arms
        leftArmAngle = self.leftArm.get()
        print(leftArmAngle)
        self.leftArm.set(-1*(self.stick2.getRawAxis(5)))        
        self.rightArm.set(-1*(self.stick2.getRawAxis(5)))



if __name__ == "__main__":
    wpilib.run(MyRobot)