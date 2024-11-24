// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Game Controller
public class ExampleSubsystem extends SubsystemBase {

  private int cnt = 0;
  private int maxcnt = 50;
  private int x = 0, y = 0;
  private int xinc = 1, yinc = 1;


  NetworkTableInstance inst;
  NetworkTable table;
 
  IntegerPublisher xPub;
  IntegerPublisher yPub;


  BooleanPublisher xButton;
  DoublePublisher xStick;

  AddressableLED myLED1;
  AddressableLEDBuffer myLEDBuffer1;

  /** Creates a new ExampleSubsystem. */
  /*public ExampleSubsystem()
  {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("frc695_test_table");

    xPub = table.getIntegerTopic("x").publish();
    xPub.set(0);

    yPub = table.getIntegerTopic("y").publish();
    yPub.set(0);

    xButton = table.getBooleanTopic("xButton").publish();
    xButton.set(false);

    xStick = table.getDoubleTopic("xStick").publish();
    xStick.set(0);

    myLED1 = new AddressableLED(2);
    myLEDBuffer1 = new AddressableLEDBuffer(5);
  }*/


  /**
   * Example command factory method.
   *
   * @return a command
   */
  /*public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          /*  xButton.set(true);
        });
  }*/

  // KNOW THIS!!!
  /*public Command testCommand1(DoubleSupplier targetvalue)
  {
    return new FunctionalCommand(


      // ** INIT
      ()-> testInit1(),
     
      // ** EXECUTE
      ()-> testExecute1(targetvalue.getAsDouble()),
     
      // ** ON INTERRUPTED
      interrupted-> testInterrupt1(),
     
      // ** END CONDITION
      ()-> testEndCondition1(),


      // ** REQUIREMENTS
      this);


  }*/

/* 
  private void testInit1()
  {
    xButton.set(true);
  }


  private void testExecute1(double value)
  {
    xStick.set(value);
    maxcnt = (int) (50 - Math.abs(value) * 50);
    //cnt = 0;
  }


  private void testInterrupt1()
  {
    xButton.set(false);
  }


  private boolean testEndCondition1()
  {
    return(false);
  }
*/


// Servo object 
/*Servo myServo1 = new Servo(0);

// Example command
  public Command servoCommand1()
  {
    return new FunctionalCommand(

      // ** INIT
      ()-> servoInit1(),
     
      // ** EXECUTE
      ()-> servoExecute1(),
     
      // ** ON INTERRUPTED
      interrupted-> servoInterrupt1(),
     
      // ** END CONDITION
      ()-> servoEndCondition1(),


      // ** REQUIREMENTS
      this);

  }


  private void servoInit1()
  {
    myLED1.setLength(myLEDBuffer1.getLength());
    for (int i = 0; i < myLEDBuffer1.getLength(); i++)
      myLEDBuffer1.setRGB(i, 0, 255, 0);
    myLED1.setData(myLEDBuffer1);
    myLED1.start();
  }


  private void servoExecute1()
  {
    myServo1.set(1);
  }


  private void servoInterrupt1()
  {
    for (int i = 0; i < myLEDBuffer1.getLength(); i++)
      myLEDBuffer1.setRGB(i, 0, 0, 0);
    myLED1.setData(myLEDBuffer1);
    myServo1.set(0.475);
  }


  private boolean servoEndCondition1()
  {
    return false;
  }

  // Task 1
  public void task1method(String button) {
    myLED1.setLength(myLEDBuffer1.getLength());
    switch (button) {
      case "x":
        for (int i = 0; i < myLEDBuffer1.getLength(); i++)
          myLEDBuffer1.setRGB(i, 0, 0, 255);
        break;
      case "y":
        for (int i = 0; i < myLEDBuffer1.getLength(); i++)
          myLEDBuffer1.setRGB(i, 255, 255, 0);
        break;
      case "a":
        for (int i = 0; i < myLEDBuffer1.getLength(); i++)
          myLEDBuffer1.setRGB(i, 0, 255, 0);
        break;
      case "b":
        for (int i = 0; i < myLEDBuffer1.getLength(); i++)
          myLEDBuffer1.setRGB(i, 255, 0, 0);
        break;
      default:
        System.out.println("default");
        for (int i = 0; i < myLEDBuffer1.getLength(); i++)
          myLEDBuffer1.setRGB(i, 0, 0, 0);
    }
    myLED1.setData(myLEDBuffer1);
    myLED1.start();
  }

  // Task 2
  public Command task2(int sec) {
    return this.startEnd(
      () -> task2method(sec),
      () -> myServo1.set(0.475));
  }
  public void task2method(int sec) {
    myServo1.set(1);
    new WaitCommand(sec);
  }

  // Task 3 
  public void task3method(String direction) {
    if (direction.equals("left"))
      myServo1.set(1);
    else myServo1.set(0);
  }

  // Task 4

  public void task4method (double value) {
    // value ranges from 0 - 1
    // servo stops at 0.475
    // SERVO
    myServo1.set(value *  0.95);
    SmartDashboard.putNumber("Servo angle", myServo1.getAngle());
    // myServo1 ranges from 0 - 0.95
    
    // Light LEDs
    myLED1.setLength(myLEDBuffer1.getLength());
    int LEDnumValue = (int) (value * 5.99); // Values 0 - 5
    double LEDdimValue = value * 2 - 1; // Values -1 - 1
    if (LEDdimValue > 0) {
      for (int i = 0; i < LEDnumValue; i++)
        myLEDBuffer1.setRGB(i, (int) (LEDdimValue * 255), 0, 0);
    }
    else if (LEDdimValue < 0) {
      LEDnumValue = (int) (LEDdimValue * -5.99);
      for (int i = 0; i < LEDnumValue; i++)
        myLEDBuffer1.setRGB(i, 0,  (int) (LEDdimValue * -255), 0);
    }

    // Rest of the LEDs
    for (int i = LEDnumValue; i < myLEDBuffer1.getLength(); i++)
      myLEDBuffer1.setRGB(i, 0, 0, 0);
    myLED1.setData(myLEDBuffer1);
    myLED1.start();
  }
    */
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  /*public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }*/


  /*@Override
  public void periodic() {


    if (DriverStation.isTeleopEnabled() == true)
    {


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
    }


  }
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}*/
