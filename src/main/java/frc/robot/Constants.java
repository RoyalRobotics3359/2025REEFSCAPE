// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public enum CanID {
    leftFront(3, 4, false), // FIX ME
    leftBack(5, 6, false), // FIX ME
    rightFront(1, 2, false), // FIX ME
    rightBack(7, 8, false), // FIX ME
    elevatorMotor1(12, Integer.MAX_VALUE, true), // Has to have encoder connected
    elevatorMotor2(11, Integer.MAX_VALUE, true), // Follower
    algaeIntakeMotor1(13, Integer.MAX_VALUE, false), // FIX ME
    alageIntakeMotor2(14, Integer.MAX_VALUE, true); // FIX ME

    private int driveID;
    private int turnID;
    private boolean isReversed;

    private CanID(int driveID, int turnID, boolean isReversed) {
      this.driveID = driveID;
      this.turnID = turnID;
      this.isReversed = isReversed;
    }

    public int getDriveID() { return driveID; }

    public int getTurnID() { return turnID; }

    public boolean isReversed() { return isReversed; }
  }

  public enum PID {
    Elevator(0.0, 0.5, 0.0, 0.0);

    private double k, p, i, d;

    private PID(double k, double p, double i, double d) {
      this.k = k;
      this.p = p;
      this.i = i;
      this.d = d;
    }

    public double getK() { return k; }

    public double getP() { return p; }

    public double getI() { return i; }

    public double getD() { return d; }
  }

  public enum Pneumatics {
    // The pneumatic IDs will follow the convention of odd numbers for IN-solenoids and even numbers for OUT-solenoids
    
    //FIX ME: Fix the names for the solenoid corresponding to the correct manipulator when finished.
    algaeIntake(3,2),
    climb1(1,0),
    climb2(5,6);

    private int in;
    private int out;

    private Pneumatics(int in, int out) {
      this.in = in;
      this.out = out;
    }

    public int getIn() { return in; }
    public int getOut() { return out; }
  }

  public enum Speeds {
    elevatorSpeed(0.60),
    algaeIntake(0.85),
    processor(0.60),
    climb(0.60),
    barge(1.00);

    private double spd;

    private Speeds(double spd) {
      this.spd = spd;
    }

    public double getSpeed() { return spd; }
  }

  // Swerve module Constants
  public static final class ModuleConstants {

    public static final int kDrivingMotorPinionTeeth = 13;

    public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0; // 5676 is the max RPM of the REV NEO Motor.
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDrivingMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

  }

  public static final class ElevatorHeightSetpoints {
    // Uses encoder values obtained from testing to correspond to heights that permit
    // the elevator to rise to specific locations without human interference.
    public static final double GRAVITY_RELEASE = 0.0; // Height = 
    public static final double FLOOR = 50.0; // Height =
  //  public static final double STAGE_ONE = 5000.0; // Height =
    public static final double STAGE_TWO = 13000.0; // Height =
    public static final double STAGE_THREE = 18000.0; // Height =
    public static final double BARGE = 17000.0; // Height =
    public static final double MAX = 19000; // Maximum travel before hardstop

    public static final double MAX_TRAVEL_INCHES = 91;
    public static final double HEIGHT_OFFSET = 4.375;
  }

  // Drivetrain Constants
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.46;
    public static final double kMaxAngularSpeed = Math.PI * 2.0;

    public static final double kTrackWidth = Units.inchesToMeters(24.497);
    public static final double kWheelBase = Units.inchesToMeters(24.497);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -1.0 * kTrackWidth / 2),
      new Translation2d(-1.0 * kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-1.0 * kWheelBase / 2, -1.0 * kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = -1.0 * Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0.0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false; // FIX ME
  }
  
  // Reasonable baeline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2.0;
  public static final double kRamseteZeta = 0.7;

  // The Robot Characterization Toolsuite provides a convinent tool for obtaining
  // these values for your robot
  public static final double ksVolts = 0.19123;
  public static final double kvVoltSecondsPerMeter = 2.8552;
}
