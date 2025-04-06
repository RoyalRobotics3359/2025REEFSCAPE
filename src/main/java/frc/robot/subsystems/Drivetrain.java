// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// REFER TO REV SWERVE MAX GITHUB FOR INFORMATION

public class Drivetrain extends SubsystemBase {

  private final SwerveModule leftFront = new SwerveModule(
    Constants.CanID.leftFront, Constants.DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule leftBack = new SwerveModule(
    Constants.CanID.leftBack, Constants.DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule rightFront = new SwerveModule(
    Constants.CanID.rightFront, Constants.DriveConstants.kFrontRightChassisAngularOffset);
  
  private final SwerveModule rightBack = new SwerveModule(
    Constants.CanID.rightBack, Constants.DriveConstants.kBackRightChassisAngularOffset);

  private final ADIS16470_IMU gyro;

  SwerveDriveOdometry odometry;

    /** Creates a new Drivetrain. */
  public Drivetrain() {
    gyro = new ADIS16470_IMU();
    gyro.calibrate();
    odometry = new SwerveDriveOdometry(
      Constants.DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
        leftFront.getPosition(),
        rightFront.getPosition(),
        leftBack.getPosition(),
        rightBack.getPosition()
      });
      

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
        leftFront.getPosition(),
        rightFront.getPosition(),
        leftBack.getPosition(),
        rightBack.getPosition()
      }
    );
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
        leftFront.getPosition(),
        rightFront.getPosition(),
        leftBack.getPosition(),
        rightBack.getPosition()
      }, pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean isFieldRelative) {
    // System.out.println("xSpeed: " + xSpeed + ", ySpeed: " + ySpeed + ", rot: " + rot);
    double xSpeedDelivered = xSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * Constants.DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
      isFieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)))
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

    leftFront.setDesiredState(swerveModuleStates[0]);
    rightFront.setDesiredState(swerveModuleStates[1]);
    leftBack.setDesiredState(swerveModuleStates[2]);
    rightBack.setDesiredState(swerveModuleStates[3]);

    // System.out.println("LF: " + leftFront.getTurnPosition() + ", RF: " + rightFront.getTurnPosition() + "LR: " + leftBack.getTurnPosition() + "RR: " + rightBack.getTurnPosition());
  }

  public void driveRobotRelative(double xSpeed, double ySpeed, double rot) {
    this.drive(xSpeed, ySpeed, rot, false);
  }
  

  public void setX() {
    leftFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)));
    rightFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-1.0 * 45.0)));
    leftBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-1.0 * 45.0)));
    rightBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    leftFront.setDesiredState(desiredStates[0]);
    rightFront.setDesiredState(desiredStates[1]);
    leftBack.setDesiredState(desiredStates[2]);
    rightBack.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    leftFront.resetEncoders();
    leftBack.resetEncoders();
    rightFront.resetEncoders();
    rightBack.resetEncoders();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate(IMUAxis.kZ) * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveDriveKinematics getKinematics() {
    return Constants.DriveConstants.kDriveKinematics;
  }

}
