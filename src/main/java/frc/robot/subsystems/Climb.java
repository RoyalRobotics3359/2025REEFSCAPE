// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  
  private TalonSRX climbMotor1;

  private DoubleSolenoid piston1;
  private DoubleSolenoid piston2;

  /** Creates a new Shoulder. */
  public Climb() {

    piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.climb1.getIn(), Constants.Pneumatics.climb1.getOut());
    piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.climb2.getIn(), Constants.Pneumatics.climb2.getOut());

    climbMotor1 = new TalonSRX(Constants.CanID.climbMotor1.getDriveID());

    climbMotor1.configFactoryDefault();

    climbMotor1.setInverted(Constants.CanID.climbMotor2.isDriveReversed());

    climbMotor1.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rotateUp() {
    climbMotor1.set(TalonSRXControlMode.PercentOutput, Constants.Speeds.climb.getSpeed());
  }

  public void rotateDown() {
    climbMotor1.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.Speeds.climb.getSpeed());
  }

  public void motorStop() {
    climbMotor1.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  // Pneumatic code to extend piston out
  public void extendShoulder() {
    piston1.set(Value.kForward);
    piston2.set(Value.kForward);
  }

  // Pneumatic code to retract piston in 
  public void retractShoulder() {
    piston1.set(Value.kReverse);
    piston2.set(Value.kReverse);

  }

  // Turns solenoid off.
  public void turnOffShoulder() {
    piston1.set(Value.kOff);
    piston2.set(Value.kOff);
  }

}
