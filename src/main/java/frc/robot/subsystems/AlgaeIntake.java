// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {

  private TalonSRX motor1;
  private TalonSRX motor2;

  private DoubleSolenoid piston;

  private DigitalInput limit;
  private int kPort = 0;
  
  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {

    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.algaeIntake.getIn(), Constants.Pneumatics.algaeIntake.getOut());

    // Defines a new motor
    motor1 = new TalonSRX(Constants.CanID.algaeIntakeMotor1.getDriveID());
    motor2 = new TalonSRX(Constants.CanID.alageIntakeMotor2.getDriveID());
    
    // Reset the motors to factory values to eliminate any preliminary issues.
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();

    // Sets the direction of the motor
    motor1.setInverted(Constants.CanID.algaeIntakeMotor1.isReversed());
    motor2.setInverted(Constants.CanID.alageIntakeMotor2.isReversed());
    
    // Sets the neutral mode of the motor to brake to prevent the intake from continuosly running
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

    limit = new DigitalInput(kPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeIn() {
    motor1.set(TalonSRXControlMode.PercentOutput, Constants.Speeds.algaeIntake.getSpeed());
    motor2.set(TalonSRXControlMode.PercentOutput, Constants.Speeds.algaeIntake.getSpeed());
  }

  public void intakeReverse() {
    motor1.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.Speeds.processor.getSpeed());
    motor2.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.Speeds.processor.getSpeed());
  } 

  public void scoreBarge() {
    motor1.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.Speeds.barge.getSpeed());
    motor2.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.Speeds.barge.getSpeed());
  }

  public void motorStop() {
    motor1.set(TalonSRXControlMode.PercentOutput, 0.0);
    motor2.set(TalonSRXControlMode.PercentOutput, 0.0);
  } 

  // These next four methods control the piston

  public void intakeExtend() {
    piston.set(Value.kForward);
  }

  public void intakeRetract() {
    piston.set(Value.kReverse);
  }

  public Command extendPiston() {
    return this.runOnce(() -> intakeExtend());
  }

  public Command retractPiston() {
    return this.runOnce(() -> intakeRetract());
  }

  public boolean isLimitPressed() {
    return !limit.get();
  }

}
