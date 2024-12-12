package frc.robot.subsystems;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedPID extends SubsystemBase{ // EXTENDS SUBSYSTEMBASE!!!!!!!!!
    private TalonFX myTalon;
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

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
        slot0Configs.kP = 0; // Proportion
        slot0Configs.kD = 0; // Derivative
        slot0Configs.kS = -1; // Friction
        slot0Configs.kV = 0; // Velocity
        slot0Configs.kA = 0; // Acceleration

        // Motion Magic (Trapezoid speed control)
        //var motionMagicConfigs = talonFXConfigs.MotionMagic;
        //talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 10; 
        //talonFXConfigs.MotionMagic.MotionMagicAcceleration = 3; 
        //motionMagicConfigs.MotionMagicJerk = 0; 

        var motorConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        myTalon.getConfigurator().apply(motorConfigs);
        myTalon.getConfigurator().apply(talonFXConfigs);
        myTalon.getConfigurator().apply(slot0Configs);

        myTalon.setPosition(0);
    }

    // Setting elevator talon to spin to a certain height
    // a, b, x, y control different set heights
    public Command talonSet(double setpoint) {
        return runOnce(() -> 
        {
            myTalon.setControl(m_request.withPosition(setpoint));
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Rotations", myTalon.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Closed Loop Output", myTalon.getClosedLoopOutput().getValueAsDouble());
    }
}
// Motion magic sysID