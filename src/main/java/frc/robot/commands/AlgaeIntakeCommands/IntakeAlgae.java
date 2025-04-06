// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {

  private AlgaeIntake intake;

  private Timer timer;

  /** Creates a new IntakeAlgae. */
  public IntakeAlgae(AlgaeIntake i) {

    intake = i;

    timer = null;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeExtend();
    intake.intakeIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.motorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake.isLimitPressed() && timer == null) {
      timer = new Timer();
      timer.start();
    }
    if (timer != null && timer.get() >= 0.5) {
      timer.stop();
      timer = null;
      return true;
    }
    return false;
  }
}
