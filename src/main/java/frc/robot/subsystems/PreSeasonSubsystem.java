package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import java.lang.Object;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import java.lang.AutoCloseable;


// Game Controller
public class PreSeasonSubsystem extends SubsystemBase {

  private final LEDSubsystem LEDs;

  /*private int cnt = 0;
  private int maxcnt = 50;
  private int x = 0, y = 0;
  private int xinc = 1, yinc = 1;

  NetworkTableInstance inst;
  NetworkTable table;
 
  IntegerPublisher xPub;
  IntegerPublisher yPub;

  BooleanPublisher xButton;
  DoublePublisher xStick;*/

  CANSparkFlex myMotor;

  // Tank + Arcade
  CANSparkMax frontLeftMotor;
  CANSparkMax frontRightMotor;
  CANSparkMax backLeftMotor;
  CANSparkMax backRightMotor;
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  // PID
  Spark myMotor2;

  /** Creates a new ExampleSubsystem. */
  public PreSeasonSubsystem()
  {
    /*inst = NetworkTableInstance.getDefault();
    table = inst.getTable("frc695_test_table");

    xPub = table.getIntegerTopic("x").publish();
    xPub.set(0);

    yPub = table.getIntegerTopic("y").publish();
    yPub.set(0);

    xButton = table.getBooleanTopic("xButton").publish();
    xButton.set(false);

    xStick = table.getDoubleTopic("xStick").publish();
    xStick.set(0);*/

    LEDs = new LEDSubsystem();

    //myMotor = new CANSparkFlex(56, MotorType.kBrushless);

    // Tank + Arcade drive: Motors
    /*frontLeftMotor = new CANSparkMax (10, MotorType.kBrushless);
    frontLeftMotor.restoreFactoryDefaults();

    frontRightMotor = new CANSparkMax (11, MotorType.kBrushless);
    frontRightMotor.restoreFactoryDefaults();

    backLeftMotor = new CANSparkMax (12, MotorType.kBrushless);
    backLeftMotor.restoreFactoryDefaults();

    backRightMotor = new CANSparkMax (13, MotorType.kBrushless);
    backRightMotor.restoreFactoryDefaults();

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);

    // Tank drive: Encoders
    leftEncoder= frontLeftMotor.getEncoder();
    leftEncoder.setPosition(0);

    rightEncoder= frontLeftMotor.getEncoder();
    rightEncoder.setPosition(0);*/

    // PID: Motor
    myMotor2 = new Spark(0);
  }

  // Single motor control: Open-loop
  public void motorSet(double angle) {
    myMotor.set(angle);
    LEDs.motorLED(angle);
  }

  public void tankSet (double leftY, double rightY) {
    frontLeftMotor.set(-1*leftY);
    frontRightMotor.set(-1*rightY);
  }

  public void arcadeSet (double Y, double X) {
    frontLeftMotor.set(Y+X);
    frontRightMotor.set(Y-X);
  }

  // SIngle motor control: Closed-loop
  public void motorSet2(double angle) {
    myMotor2.set(angle);
  }

@Override
  public void periodic() {

    if (DriverStation.isTeleopEnabled() == true) {

/* 
    // This method will be called once per scheduler run
    if (++cnt == maxcnt)
    {

      cnt = 0;
      x = x + xinc;
      if (x == 40)
      {
        xinc = xinc * -1;
      }
      if (x == 0)
      {
        xinc = xinc * -1;
      }
      y = y + yinc;
      if (y == 30)
      {
        yinc = yinc * -1;
      }
      if (y == 0)
      {
        yinc = yinc * -1;
      }
      xPub.set(x);
      yPub.set(y);
    }*/

    }
  }
}
