// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

  private double ty = 0.0;
  private boolean hasTarget = false;

  private double limelightMountAngleDegrees = 90.0;

  // Height of the limelight lens to the floor
  private double limelightLensHeightInches = 0.0;

  private double angleToGoalDegrees = 0.0;
  private double angleToGoalRadians = 0.0;

  // The height in inches of the april tags relative to these field elements.
  // Information obtained from:
  // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
  private double PROCESSOR_HEIGHT = 47.88;
  private double BARGE_HEIGHT = 70.73;
  private double REEF_HEIGHT = 8.75;

  /** Creates a new Limelight. */
  public Limelight() {
    LimelightHelpers.setLEDMode_PipelineControl("");
    LimelightHelpers.setLEDMode_ForceOff("");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ty = LimelightHelpers.getTY("");
    angleToGoalDegrees = limelightMountAngleDegrees + ty;
    angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
  }

  public double getDistanceFromTarget(double lensHeight, double goalHeight) {
    limelightLensHeightInches = lensHeight;
    return (goalHeight-limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }


  // FIX THE Integer.MAX_VALUE to actual height after getting the linear function from the elevator encoder readings
  
  public double getDistanceToProcessor() {
    return getDistanceFromTarget(Integer.MAX_VALUE, PROCESSOR_HEIGHT);
  }

  public double getDistanceToBarge() {
    return getDistanceFromTarget(Integer.MAX_VALUE, BARGE_HEIGHT);
  }

  public double getDistanceToReef() {
    return getDistanceFromTarget(Integer.MAX_VALUE, REEF_HEIGHT);
  }
}
