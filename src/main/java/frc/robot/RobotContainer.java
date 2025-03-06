// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AlgaeIntakeCommands.IntakeAlgae;
import frc.robot.commands.AlgaeIntakeCommands.OutakeAlgae;
import frc.robot.subsystems.AlgaeIntake;

// import frc.robot.Constants.OperatorConstants;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Drivetrain;

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

//   // The robot's subsystems and commands are defined here...
//   // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

//   // Replace with CommandPS4Controller or CommandJoystick if needed
//   private final CommandXboxController m_driverController =
//       new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(OperatorConsole oc, Drivetrain d, AlgaeIntake i) {

    console = oc;
    drive = d;
    intake = i;

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

    console.getGameController().a().whileTrue(new IntakeAlgae(intake));
    console.getGameController().b().whileTrue(new OutakeAlgae(intake));

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
