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
    //private final LEDSubsystem LEDs = new LEDSubsystem();

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
    private double setPoint;


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

        myMotor2.restoreFactoryDefaults();
        myEncoder.setPosition(0);

        myPID = myMotor2.getPIDController();
            myPID.setP(kP);
            myPID.setD(kD);
            // Set the minimum and maximum outputs of the motor [-1, 1]
            myPID.setOutputRange(kMinOutput, kMaxOutput);
            // Set kFF
            myPID.setFF(1/Kv);
    }

    public void BasicDrivetrains () {
        
        //tankDrive.tankSet(leftForwardSpeed, rightForwardSpeed);

        //arcadeDrive.arcadeSet(forwardSpeed * 0.7, turningSpeed * 0.3);
    }

    public void PID (int value) {
        switch (value) {
            case 1:
                setPoint = 0;
                //LEDs.setColor("b");
                break;
            case 2:
                setPoint = 50;
                //LEDs.setColor("y");
                break;
            case 3:
                setPoint = -100;
                //LEDs.setColor("r");
                break;
            case 4:
                setPoint = 100;
                //LEDs.setColor("g");
                break;
        }
        myPID.setReference(setPoint, CANSparkFlex.ControlType.kPosition);
        //robotContainer.getController().b().whileTrue(() -> 
        //myMotor2.restoreFactoryDefaults(),
        //myEncoder.setPosition(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder", myEncoder.getPosition());
        SmartDashboard.putNumber("Setpoint", setPoint);
    }

    public RelativeEncoder getEncoder() {
        return myEncoder;
    }
}