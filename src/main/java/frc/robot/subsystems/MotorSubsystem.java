package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.subsystems.PreSeasonSubsystem;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    private final PreSeasonSubsystem tankDrive;
    private final PreSeasonSubsystem arcadeDrive;
    private final PreSeasonSubsystem swerveDrive;

    // Tank drive
    private double leftForwardSpeed;
    private double rightForwardSpeed; 

    // Arcade drive
    private double forwardSpeed;
    private double turningSpeed;

    // PID
    private Spark myMotor2;
    private Encoder myEncoder;
    private final double kMotor = 0.01;
    private double setpoint = 0;

    public MotorSubsystem(PreSeasonSubsystem driveTrains) {
        tankDrive = driveTrains;
        arcadeDrive = driveTrains;
        swerveDrive = driveTrains;

        // Tank drive
        //leftForwardSpeed = RobotContainer.myLeftJoystick.getY();
        //rightForwardSpeed = RobotContainer.myRightJoystick.getY();

        // Arcade drive
        //forwardSpeed = RobotContainer.getController().getLeftY();
        //turningSpeed = RobotContainer.getController().getRightX();

        // PID
        myMotor2 = new Spark(0);
        myEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    }

    public void BasicDrivetrains () {
        
        //tankDrive.tankSet(leftForwardSpeed, rightForwardSpeed);

        //arcadeDrive.arcadeSet(forwardSpeed * 0.7, turningSpeed * 0.3);
    }

    public Command PID () {
        return new FunctionalCommand(
            ()-> PIDinit(),

            ()-> PIDexecute(),

            interrupted-> {},

            ()-> false,

            this);
    }

    private void PIDinit () {
        myEncoder.reset();
        setpoint = 0;
    }

    private void PIDexecute () {
        // Proportion
        double sensorPosition = myEncoder.get();
        double error = setpoint - sensorPosition;
        double outputSpeed = kMotor * error;
        myMotor2.set(outputSpeed);

        // Derivative
    }
}
