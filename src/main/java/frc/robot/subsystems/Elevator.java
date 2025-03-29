// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private TalonSRX elevatorMotor1;
    private TalonSRX elevatorMotor2;

    /** Creates a new Lift. */
    public Elevator() {
        elevatorMotor1 = new TalonSRX(Constants.CanID.elevatorMotor1.getDriveID()); // This motor has the encoder connected to it.
        elevatorMotor2 = new TalonSRX(Constants.CanID.elevatorMotor2.getDriveID()); 
        

        elevatorMotor1.setInverted(Constants.CanID.elevatorMotor1.isReversed());
        elevatorMotor2.setInverted(Constants.CanID.elevatorMotor2.isReversed());

        elevatorMotor1.setNeutralMode(NeutralMode.Brake);
        elevatorMotor2.setNeutralMode(NeutralMode.Brake);

        // The master has to have the encoder.  The follower cannot have the encoder
        elevatorMotor2.follow(elevatorMotor1);

        elevatorMotor1.setSensorPhase(true);

        configPidControl();        
    }
    
    /**
     * Configure the master motor controller for closed-loop (PID) control using
     * the quadrature encoder for the position
     */
    private void configPidControl() {
      elevatorMotor1.config_kF(0, Constants.PID.Elevator.getK());
      elevatorMotor1.config_kP(0, Constants.PID.Elevator.getP());
      elevatorMotor1.config_kI(0, Constants.PID.Elevator.getI());
      elevatorMotor1.config_kD(0, Constants.PID.Elevator.getD());

      elevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1000);
      elevatorMotor1.configReverseSoftLimitThreshold(Constants.ElevatorHeightSetpoints.FLOOR);
      elevatorMotor1.configReverseSoftLimitEnable(true);
      elevatorMotor1.configForwardSoftLimitThreshold(Constants.ElevatorHeightSetpoints.MAX);
      elevatorMotor1.configForwardSoftLimitEnable(true);
    }

    @Override
    public void periodic() { 
      double encoderPosition = getEncoderPosition();
      SmartDashboard.putNumber("Elevator Position", encoderPosition);
      SmartDashboard.putNumber("Elevator 1 Voltage", elevatorMotor1.getMotorOutputVoltage());
      SmartDashboard.putNumber("Elevator 2 Voltage", elevatorMotor2.getMotorOutputVoltage());
    }

    // Gives power to a motor in the decimal for of a percent (EX: 10% = 0.1) ranging from [-1,1].
    public void setPercentPower(double power)  {
      elevatorMotor1.set(TalonSRXControlMode.PercentOutput, power);
    }

    // The next two functions are similar to the previous; however, they use a predefined value in the Constants.java file for lifting and depressing the elevator.
    public void extendLift () {
      elevatorMotor1.set(TalonSRXControlMode.PercentOutput, Constants.Speeds.elevatorSpeed.getSpeed());
    }

    public void retarctLift() {
      elevatorMotor1.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.Speeds.elevatorSpeed.getSpeed());
    }

    public void setLiftStage(double stage) {
      this.configPidControl();
      elevatorMotor1.set(TalonSRXControlMode.Position, stage);
    }

    // Default function to stop the motors
    public void motorStop () {
      elevatorMotor1.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    public double getEncoderPosition() {
      double encoderPosition = elevatorMotor1.getSelectedSensorPosition(0);
      return encoderPosition;
    }

    public Command resetEncoder() {
      return this.runOnce(() -> elevatorMotor1.setSelectedSensorPosition(0));
    }

    public double getElevatorPositionPercent() {
      double p = this.getEncoderPosition();
      if (p < 0) {
        p = 0;
      } else if (p > Constants.ElevatorHeightSetpoints.MAX) {
        p = Constants.ElevatorHeightSetpoints.MAX;
      }
      return p / Constants.ElevatorHeightSetpoints.MAX;
    }

    public double getEstimatedHeightFromGround() {
      return (this.getElevatorPositionPercent() * Constants.ElevatorHeightSetpoints.MAX_TRAVEL_INCHES) + Constants.ElevatorHeightSetpoints.HEIGHT_OFFSET;
    }

  } 