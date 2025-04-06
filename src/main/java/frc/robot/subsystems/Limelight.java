// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class Limelight extends SubsystemBase {

  private Elevator elevator = new Elevator();

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
  private Drivetrain drivetrain;


  /** Creates a new Limelight. */
  public Limelight(Drivetrain drivetrain) {
    LimelightHelpers.setLEDMode_PipelineControl("");
    LimelightHelpers.setLEDMode_ForceOff("");
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ty = LimelightHelpers.getTY("");
    angleToGoalDegrees = limelightMountAngleDegrees + ty;
    angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    SmartDashboard.putBoolean("processor range", getDistanceToProcessor() <= 16.0);
    if(!Robot.isSimulation()) updatePoseEstimation();
  }

  public double getDistanceFromTarget(double goalHeight) {
    limelightLensHeightInches = elevator.getEstimatedHeightFromGround();
    return (goalHeight-limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }


  // FIX THE Integer.MAX_VALUE to actual height after getting the linear function from the elevator encoder readings

  public double getDistanceToProcessor() {
    return getDistanceFromTarget(PROCESSOR_HEIGHT);
  }

  public double getDistanceToBarge() {
    return getDistanceFromTarget(BARGE_HEIGHT);
  }

  public double getDistanceToReef() {
    return getDistanceFromTarget(REEF_HEIGHT);
  }
  public boolean visionUpdates = false;
  private final RectanglePoseArea fieldBoundary = new RectanglePoseArea(new Translation2d(0,0),
    new Translation2d(17.55,8.05));

    public void updatePoseEstimation() {
      try{
if (visionUpdates && LimelightHelpers.getTV("")){
  AprilTagFieldLayout Tagfield = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);//k2025ReefscapeAndyMark
  Pose2d tagPose = new Pose2d();
  Optional<Pose3d> tagPosePre = Optional.of(Tagfield.getTagPose((int) LimelightHelpers.getFiducialID((""))).get());
  if (tagPosePre.isPresent()) {
    tagPose = tagPosePre.get().toPose2d();

    LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT2 = LimelightHelpers
    .getBotPoseEstimate_wpiBlue_MegaTag2("");
    LimelightHelpers.PoseEstimate limelightBotPoseEstimateMT = LimelightHelpers
    .getBotPoseEstimate_wpiBlue("");

    LimelightHelpers.PoseEstimate poseToUse = limelightBotPoseEstimateMT;

    if (fieldBoundary.isPoseWithinArea(poseToUse.pose) && poseToUse.tagCount > 0) {

      if (limelightBotPoseEstimateMT.avgTagDist < Units.feetToMeters(12)) {
        poseToUse = limelightBotPoseEstimateMT;
      } else {
        poseToUse = limelightBotPoseEstimateMT2;
      } 
      Pose2d finalPose = new Pose2d(poseToUse.pose.getX(), poseToUse.pose.getY(),
      poseToUse.pose.getRotation());



    }
  }
}
      }
      catch (Exception e) {}
    }
}
