// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorConsole;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JoystickDrive extends Command {

  private Drivetrain drive;
  private OperatorConsole console;
  private Elevator elevator;

  /** Creates a new JoystickDrive. */
  public JoystickDrive(Drivetrain drive, Elevator elevator,OperatorConsole console) {
    
    this.drive = drive;
    this.elevator = elevator;
    this.console = console;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double spd = console.getDriveController().getLeftStickY();
    double stf = -1.0 * console.getDriveController().getLeftStickX();
    double rot = -1.0 * console.getDriveController().getRightStickX();

    if (elevator.getElevatorPositionPercent() >= 0.5) {
      spd = spd * 0.50;
    }
    
    drive.drive(spd, stf, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
