// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

// REFER TO REV SWERVE MAX GITHUB FOR INFORMATION

public class SwerveModule extends SubsystemBase {

  private SparkMax driveMotor;
  private SparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private AbsoluteEncoder turnEncoder;

  private SparkClosedLoopController driveClosedLoopController;
  private SparkClosedLoopController turnClosedLoopController;

  private double chassisAngularOffset = 0.0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** Creates a new SwerveModule. */
  public SwerveModule(Constants.CanID position, double chassisAngularOffset) {

    driveMotor = new SparkMax(position.getDriveID(), MotorType.kBrushless);
    turnMotor = new SparkMax(position.getTurnID(), MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    driveClosedLoopController = driveMotor.getClosedLoopController();
    turnClosedLoopController = turnMotor.getClosedLoopController();

    driveMotor.configure(Configs.SwerveModule.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(Configs.SwerveModule.turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveMotor.setInverted(position.isReversed());
    turnMotor.setInverted(position.isReversed());

    this.chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), 
          new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(),
      new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));
    
    driveClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void resetEncoders() { driveEncoder.setPosition(0.0); }

  public double getTurnPosition() {
    return turnEncoder.getPosition();
  }
}
