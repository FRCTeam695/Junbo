package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.subsystems.PreSeasonSubsystem;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    private static double setpointPosition;
    private static double setpointVelocity;
    private static double kP = 0;
    private static double kD = 0;
    private static double Kv = 0;

    private static final double kMaxOutput = 1;
    private static final double kMinOutput = -1;

    //private ShuffleboardTab tab = Shuffleboard.getTab("Encoder");
    //private GenericEntry encoderTab = tab.add("???", 0).getEntry();

    public MotorSubsystem(PreSeasonSubsystem driveTrains, int motorNum) {
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
        //myEncoder.setPosition(0);

        myPID = myMotor2.getPIDController();
            myPID.setP(kP); // Set kP
            myPID.setD(kD); // Set kD
            myPID.setOutputRange(kMinOutput, kMaxOutput); // Set the minimum and maximum outputs of the motor [-1, 1]
            myPID.setFF(Kv); // Set kFF
        
        Preferences.initDouble("Encoder Setpoint Position", 0); // Setpoint Position
        Preferences.initDouble("Encoder Setpoint Velocity", 0); // Setpoint Velocity
        Preferences.initDouble("kP Value", kP); // kP
        Preferences.initDouble("kD Value", kD); // kD
        Preferences.initDouble("kv Value", Kv); // kv
    }

    public void BasicDrivetrains () {
        
        //tankDrive.tankSet(leftForwardSpeed, rightForwardSpeed);

        //arcadeDrive.arcadeSet(forwardSpeed * 0.7, turningSpeed * 0.3);
    }

    public void PID (int pv/*int value*/) {
        
        //SmartDashboard.putNumber("Encoder Setpoint", 0);
        if (pv == 1)
            myPID.setReference(setpointPosition, CANSparkFlex.ControlType.kPosition);
        if (pv == 2)
            myPID.setReference(setpointVelocity, CANSparkFlex.ControlType.kVelocity);
        //robotContainer.getController().b().whileTrue(() -> 
        //myMotor2.restoreFactoryDefaults(),
        //myEncoder.setPosition(0));
    }

    public RelativeEncoder getEncoder() {
        return myEncoder;
    }

    public Command betterPID(DoubleSupplier speed){
        return 
        runOnce(() -> myMotor2.restoreFactoryDefaults()).andThen(
        run(
            ()-> {
                myPID.setReference(speed.getAsDouble() * 5700, CANSparkFlex.ControlType.kVelocity);
                // Position control
                SmartDashboard.putNumber("Encoder Position", myEncoder.getPosition());
                setpointPosition = Preferences.getDouble("Encoder Setpoint Position", setpointPosition);

                // Velocity control
                SmartDashboard.putNumber("Encoder Velocity", myEncoder.getVelocity());
                setpointVelocity = Preferences.getDouble("Encoder Setpoint Velocity", setpointVelocity);

                kP = Preferences.getDouble("kP Value", kP);
                kD = Preferences.getDouble("kD Value", kD);
                Kv = Preferences.getDouble("kv Value", Kv);
                myPID.setP(kP); // Set kP
                myPID.setD(kD); // Set kD
                myPID.setFF(Kv);
            }
        ));
    }

    

}

/*@Override
    public void periodic() {
        // Position control
        SmartDashboard.putNumber("Encoder Position", myEncoder.getPosition());
        setpointPosition = Preferences.getDouble("Encoder Setpoint Position", setpointPosition);

        // Velocity control
        SmartDashboard.putNumber("Encoder Velocity", myEncoder.getVelocity());
        setpointVelocity = Preferences.getDouble("Encoder Setpoint Velocity", setpointVelocity);

        kP = Preferences.getDouble("kP Value", kP);
        kD = Preferences.getDouble("kD Value", kD);
        Kv = Preferences.getDouble("kv Value", Kv);
        myPID.setP(kP); // Set kP
        myPID.setD(kD); // Set kD
        myPID.setFF(Kv);
    }
*/