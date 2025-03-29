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

  private DoubleSolenoid piston1;
  // private DoubleSolenoid piston2;

  /** Creates a new Shoulder. */
  public Climb() {

    piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.climb1.getIn(), Constants.Pneumatics.climb1.getOut());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Pneumatic code to extend piston out
  public void extendPiston() {
    piston1.set(Value.kForward);
  }

  // Pneumatic code to retract piston in 
  public void retractPiston() {
    piston1.set(Value.kReverse);

  }

  // Turns solenoid off.
  public void turnOffPiston() {
    piston1.set(Value.kOff);
    // piston2.set(Value.kOff);
  }

}
