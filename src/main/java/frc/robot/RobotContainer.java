// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.PushbackInputStream;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.WheelLock;
import frc.robot.commands.AlgaeIntakeCommands.IntakeAlgae;
import frc.robot.commands.AlgaeIntakeCommands.OutakeAlgae;
import frc.robot.commands.AlgaeIntakeCommands.ScoreBarge;
import frc.robot.commands.ClimbCommands.Pull;
import frc.robot.commands.ClimbCommands.Push;
import frc.robot.commands.ElevatorCommands.ElevatorDepress;
import frc.robot.commands.ElevatorCommands.ElevatorRise;
import frc.robot.commands.ElevatorCommands.SetElevatorStage;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;

// import frc.robot.Constants.OperatorConstants;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private OperatorConsole console;
  private Drivetrain drive;
  private AlgaeIntake intake;
  private Climb climb;
  private Elevator elevator;

//   // The robot's subsystems and commands are defined here...
//   // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

//   // Replace with CommandPS4Controller or CommandJoystick if needed
//   private final CommandXboxController m_driverController =
//       new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(OperatorConsole oc, Drivetrain d, AlgaeIntake i, Climb c, Elevator e) {

    console = oc;
    drive = d;
    intake = i;
    climb = c;
    elevator = e;

    // Add comands to PathPlanner for use in autonomous


    // Configure the trigger/button bindings
    configureBindings();
  }

//   /**
//    * Use this method to define your trigger->command mappings. Triggers can be created via the
//    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
//    * predicate, or via the named factories in {@link
//    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
//    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
//    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
//    * joysticks}.
//    */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    // BUTTON MAPPTING EXAMPLE
    // console.m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    /* Bind buttons to game controller */
    console.getGameController().rightTrigger().whileTrue(new IntakeAlgae(intake));
    console.getGameController().leftTrigger().whileTrue(new OutakeAlgae(intake));

    console.getGameController().a().whileTrue(new ScoreBarge(intake));
    console.getGameController().b().whileTrue(intake.extendPiston());

    console.getGameController().back().whileTrue(intake.retractPiston());
    console.getGameController().start().whileTrue(elevator.resetEncoder());

    console.getGameController().rightBumper().whileTrue(new ElevatorRise(elevator, 0));
    console.getGameController().leftBumper().whileTrue(new ElevatorDepress(elevator));

    console.getGameController().dPadRight().whileTrue(
      new SetElevatorStage(elevator, intake, Constants.ElevatorHeightSetpoints.STAGE_TWO));
    console.getGameController().dPadUp().whileTrue(
      new SetElevatorStage(elevator, intake, Constants.ElevatorHeightSetpoints.STAGE_THREE));
    console.getGameController().dPadLeft().whileTrue(
      new SetElevatorStage(elevator, intake, Constants.ElevatorHeightSetpoints.MAX));
    console.getGameController().dPadDown().whileTrue(
      new SetElevatorStage(elevator, intake, Constants.ElevatorHeightSetpoints.FLOOR));

    console.getGameController().x().whileTrue(new Push(climb));
    console.getGameController().y().whileTrue(new Pull(climb));
    
    /* Bind buttons on drive controller */
    console.getDriveController().x().whileTrue(new WheelLock(drive, console));

    // console.getDriveController().x().whileTrue(new 
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   // return Autos.exampleAuto(m_exampleSubsystem);
  // }

  // private static Command createCommand(Drivetrain drive, Trajectory trajectory)
  // {
  //   RamseteController ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
  //   SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter);

  //   Command command = null;
  //   // command = new SwerveControllerCommand(
  //   //     trajectory,
  //   //     drive::getPose,
  //   //     drive.getKinematics(),
  //   //     drive.getXPidController(),
  //   //     drive.getYPidController(),
  //   //     drive.getThetaController(),
  //   //     drive.getSwerveStates(),
  //   //     drive
  //   // );
  //   return command;
  // }
}
