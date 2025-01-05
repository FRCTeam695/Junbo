package frc.robot.subsystems;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedPID extends SubsystemBase{ // EXTENDS SUBSYSTEMBASE!!!!!!!!!
    private TalonFX myTalon;
    private MotionMagicVoltage m_request;
    private TrapezoidProfile m_profile;

    public AdvancedPID() {
        myTalon = new TalonFX(50);

        var talonFXConfigs = new TalonFXConfiguration();
        m_request = new MotionMagicVoltage(0);

        // Limits
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set neutral mode
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 15; // Amps

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 35; // Rotations
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        
        // Tuning
        // Mainly for overshoot + undershoot tuning
        var slot0Configs = talonFXConfigs.Slot0;
        // 0.5V is needed for gravity/friction
        slot0Configs.kG = 0.37; // Gravity 0.37 volt
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kS = 0.13; // Friction 0.13 volt
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        slot0Configs.kV = 0.1; // volt/rps
        slot0Configs.kA = 0.01; // volt/rps/s
        slot0Configs.kP = 3.5; // volt/(r*s)
        slot0Configs.kD = 0.1; // volt/rps

        // Motion Magic (Trapezoid speed control)
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60; 
        motionMagicConfigs.MotionMagicAcceleration = 200; 
        motionMagicConfigs.MotionMagicJerk = 2000;

        myTalon.getConfigurator().apply(talonFXConfigs);

        myTalon.setPosition(0);  
    }

    // Setting elevator talon to spin to a certain height
    // a, b, x, and right bumper control different set heights
    public Command talonSet(double setpoint) {
        return runOnce(() -> 
        {
            myTalon.setControl(m_request.withPosition(setpoint));
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Rotations", myTalon.getPosition(true).getValueAsDouble());
        SmartDashboard.putNumber("Closed Loop Output", myTalon.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("FF Output", myTalon.getClosedLoopFeedForward().getValueAsDouble());
        SmartDashboard.putNumber("Velocity", myTalon.getClosedLoopFeedForward().getValueAsDouble());
        
    }
}
// Motion magic sysID