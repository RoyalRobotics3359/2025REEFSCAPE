// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

// REFER TO REV SWERVE MAX GITHUB FOR INFORMATION

/** Add your docs here. */
public class Configs {
    public static final class SwerveModule {

        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();

        static {
            double driveFactor = Constants.ModuleConstants.kWheelCircumferenceMeters * Math.PI / Constants.ModuleConstants.kDrivingMotorReduction;
            double turnFactor = 2.0 * Math.PI;
            double driveVelocityFeedForward = 1.0 / Constants.ModuleConstants.kDriveWheelFreeSpeedRps;

            driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            driveConfig.encoder.positionConversionFactor(driveFactor).velocityConversionFactor(driveFactor / 60.0);
            driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.1,0.00,0).velocityFF(driveVelocityFeedForward).outputRange(-1.0, 1.0); //FIX PID CONTROLLER

            turnConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(17 /* 20 */);
            turnConfig.absoluteEncoder.inverted(true).positionConversionFactor(turnFactor).velocityConversionFactor(turnFactor / 60.0);
            turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1.01,0.00,0.00).outputRange(-1.0, 1.0).positionWrappingEnabled(true).positionWrappingInputRange(0, turnFactor); //FIX PID CONTROLLER
        }
    }
}
