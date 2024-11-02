package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.subsystems.PreSeasonSubsystem;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private CANSparkFlex myMotor2;
    private RelativeEncoder myEncoder;

    private SparkPIDController myPID;
    private final double setPoint = 1000.0;


    public MotorSubsystem(PreSeasonSubsystem driveTrains, int motorNum, 
                          double kP, double kD, double Kv, 
                          double kMinOutput, double kMaxOutput) {
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
        myMotor2 = new CANSparkFlex(motorNum, MotorType.kBrushless);
        myEncoder =  myMotor2.getEncoder();
        myPID = myMotor2.getPIDController();
            myPID.setP(1);
            myPID.setD(0.1);
            // Set the minimum and maximum outputs of the motor [-1, 1]
            myPID.setOutputRange(kMinOutput, kMaxOutput);
            // Set kFF
            myPID.setFF(1/Kv);

        System.out.println("???");
    }

    public void BasicDrivetrains () {
        
        //tankDrive.tankSet(leftForwardSpeed, rightForwardSpeed);

        //arcadeDrive.arcadeSet(forwardSpeed * 0.7, turningSpeed * 0.3);
    }

    public void PID () {
        SmartDashboard.putNumber("Encoder", myEncoder.getPosition());
        myPID.setReference(setPoint, CANSparkFlex.ControlType.kPosition);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("myString", myEncoder.getPosition());
    }

    public RelativeEncoder getEncoder() {
        return myEncoder;
    }
}