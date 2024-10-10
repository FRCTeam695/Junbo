// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.PreSeasonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PreSeasonCommands extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PreSeasonSubsystem tankDrive;
  private final PreSeasonSubsystem arcadeDrive;
  private final PreSeasonSubsystem swerveDrive;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PreSeasonCommands(PreSeasonSubsystem subsystem) {
    tankDrive = subsystem;
    arcadeDrive = subsystem;
    swerveDrive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting joystick drive command!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tank drive
    //double leftForwardSpeed = RobotContainer.myLeftJoystick.getY();
    //double rightForwardSpeed = RobotContainer.myRightJoystick.getY();
    //tankDrive.tankSet(leftForwardSpeed, rightForwardSpeed);

    // Arcade drive
    double forwardSpeed = RobotContainer.getController().getLeftY();
    double turningSpeed = RobotContainer.getController().getRightX();
    arcadeDrive.arcadeSet(forwardSpeed * 0.7, turningSpeed * 0.3);
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
