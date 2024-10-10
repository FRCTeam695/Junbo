// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.PreSeasonCommands;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PreSeasonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj2.command.Commands.*;


//import edu.wpi.first.wpilibj.PWM;
//import edu.wpi.first.wpilibj.Servo;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final PreSeasonSubsystem mySubsystem = new PreSeasonSubsystem();
  private final PreSeasonCommands driveTrain = new PreSeasonCommands(mySubsystem); // mySubsystem = PreSeasonSubsystem

  // Joysticks (Not controller)
  //public static CommandJoystick myLeftJoystick;
  //public static CommandJoystick myRightJoystick;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController m_driverController =
      new CommandXboxController(0);

  public static CommandXboxController getController() {
    return m_driverController;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Joystick buttons (Not controller)
    //myLeftJoystick = new CommandJoystick(0);
    //myRightJoystick = new CommandJoystick(1);
  
    // Configure the trigger bindings
    configureBindings();
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's _ button is pressed,
    // cancelling on release.

    // Example control
    // m_driverController.x().whileTrue(m_exampleSubsystem.servoCommand1());

    // Task 1 - DONE  
    /*m_driverController.x().onTrue(runOnce(() -> m_exampleSubsystem.task1method("x"), m_exampleSubsystem));
    m_driverController.y().onTrue(runOnce(() -> m_exampleSubsystem.task1method("y"), m_exampleSubsystem));
    m_driverController.a().onTrue(runOnce(() -> m_exampleSubsystem.task1method("a"), m_exampleSubsystem));
    m_driverController.b().onTrue(runOnce(() -> m_exampleSubsystem.task1method("b"), m_exampleSubsystem));*/

    // Task 2 - In progress
    //m_driverController.x().whileTrue(m_exampleSubsystem.task2(3));

    // Task 3 - DONEish
    //m_driverController.leftBumper().whileTrue(runOnce(() -> m_exampleSubsystem.task3method("left"), m_exampleSubsystem));
    //m_driverController.rightBumper().whileTrue(runOnce(() -> m_exampleSubsystem.task3method("right"), m_exampleSubsystem));

    // Task 4 - In progress
    /*m_driverController.x().whileActiveOnce(
      runOnce(() -> m_exampleSubsystem.task4method((m_driverController.getRightY() + 1)/2), m_exampleSubsystem));
      */
      //m_exampleSubsystem.setDefaultCommand(runOnce(() -> m_exampleSubsystem.task4method((m_driverController.getRightY() + 1)/2), m_exampleSubsystem));
    
    // Motor movement
    // mySubsystem.setDefaultCommand(runOnce(() -> mySubsystem.motorSet(m_driverController.getRightY()), mySubsystem));

    mySubsystem.setDefaultCommand(driveTrain);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveTrain;
  }
}
