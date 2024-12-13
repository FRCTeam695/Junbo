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
    private PositionVoltage m_request;
    private TrapezoidProfile m_profile;

    public AdvancedPID() {
        myTalon = new TalonFX(50);

        var talonFXConfigs = new TalonFXConfiguration();

        // Limits
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set neutral mode
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 15; // Amps

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 30; // Rotations
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        
        // PDSAV Settings
        // Mainly for overshoot + undershoot tuning
        var slot0Configs = new Slot0Configs();
        // 0.5V is neede
        slot0Configs.kG = 0.20; // Gravity 0.2
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kS = 0.30; // Friction 0.3
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        slot0Configs.kV = 1; // Velocity
        slot0Configs.kA = 0; // Acceleration
        slot0Configs.kP = 0; // Proportion
        slot0Configs.kD = 0; // Derivative

        m_profile= new TrapezoidProfile(
            new TrapezoidProfile.Constraints(15, 30));
        // Motion Magic (Trapezoid speed control)
        //var motionMagicConfigs = talonFXConfigs.MotionMagic;
        //talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 10; // 10 rot/sec
        //talonFXConfigs.MotionMagic.MotionMagicAcceleration = 20; // 0.5 sec to reach cruise v
        //motionMagicConfigs.MotionMagicJerk = 0;

        var motorConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        myTalon.getConfigurator().apply(motorConfigs);
        myTalon.getConfigurator().apply(talonFXConfigs);
        myTalon.getConfigurator().apply(slot0Configs);
        //myTalon.getConfigurator().apply(motionMagicConfigs);

        myTalon.setPosition(0);

        m_request= new PositionVoltage(0).withSlot(0);
    }

    // Setting elevator talon to spin to a certain height
    // a, b, x, y control different set heights
    public Command talonSet(double setpoint) {
        return runOnce(() -> 
        {
            TrapezoidProfile.State m_goal = new TrapezoidProfile.State(setpoint, 0);
            TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();;;
            m_setpoint = m_profile.calculate(3, m_setpoint, m_goal);
            m_request.Position = m_setpoint.position;
            m_request.Velocity = m_setpoint.velocity;
            myTalon.setControl(m_request);
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