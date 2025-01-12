// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}

/*Hello, this is greetings from a software student in team 695 Bison Robotics. Recently I finished manually tuning an old elevator setup, but it could be improved by sysID. I wrote the code for data logging (e.g. Quasistatic & Dynamic motion profiles, SignalLogger.start() & SignalLogger.stop() methods), and attempted to extract the log files. 
Here is where I ran into my first issue: while using a USB flash drive, and with data logging enabled, no data was passed into the USB. On Phoenix 6 Tuner X (I use Phoenix 6 kraken) the USB was present, but no matter what I did no data seemed to be able to reach that roboRio-connected USB. I could not resolve this USB issue. 
Next, I switched to using the roboRio storage, and data did appear on Phoenix tuner. However, as I try to scan for signals in the file, it gave an error: "Failed to update available signals: Failed to parse tools index. Please try converting this log while connect to the internet and verify that no proxy is enabled." My internet connection was secure, but again this error kept appearing when I try to scan for signals. This was the second error I ran into. 
Lastly, when I attempt to convert the hoot file into wpilog, the finished file was nowhere to be found. Phoenix tuner even says completed, but again nothing appeared in my files. This was the third unresolved issue relating to sysID. 
I hope I gave enough details about these minor phoenix tuner issues. I hope the problems can be resolved soon! */