// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

/** Add your docs here. */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private TalonSRX elevatorMotor;
    /** Creates a new Lift. */
    public Elevator() {
        elevatorMotor = new TalonSRX(Constants.CanID.elevatorMotor.getDriveID()); 

        elevatorMotor.configFactoryDefault();

        elevatorMotor.setInverted(Constants.CanID.elevatorMotor.isDriveReversed());

        elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    @Override
    public void periodic() {
    
    }

    // Gives power to a motor in the decimal for of a percent (EX: 10% = 0.1) ranging from [-1,1].
    public void setPercentPower(double power)  {
      elevatorMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    // The next two functions are similar to the previous; however, they use a predefined value in the Constants.java file for lifting and depressing the elevator.
    public void extendLift () {
      elevatorMotor.set(TalonSRXControlMode.PercentOutput, Constants.Speeds.elevatorSpeed.getSpeed());
    }

    public void retarctLift() {
      elevatorMotor.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.Speeds.elevatorSpeed.getSpeed());
    }

    // Default function to stop the motors
    public void motorStop () {
      elevatorMotor.set(TalonSRXControlMode.PercentOutput, 0.0);

    }
  } 