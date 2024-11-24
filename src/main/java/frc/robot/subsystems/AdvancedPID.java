package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvancedPID extends SubsystemBase{ // EXTENDS SUBSYSTEMBASE!!!!!!!!!
    private TalonFX myTalon;

    private MotionMagicVoltage voltage;

    public AdvancedPID() {
        myTalon = new TalonFX(50);

        var talonFXConfigs = new TalonFXConfiguration();

        // Limits
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set neutral mode
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 15; // Amps
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; // Rotations
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        
        // PDSAV Settings
        // Mainly for overshoot + undershoot tuning
        var configs = talonFXConfigs.Slot0;
        configs.kP = 0; // Proportion
        configs.kD = 0; // Derivative
        configs.kS = 0; // Friction
        configs.kV = 0; // Velocity
        configs.kA = 0; // Acceleration

        // Motion Magic (Trapezoid speed control)
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // dx
        motionMagicConfigs.MotionMagicAcceleration = 0; // ddx
        motionMagicConfigs.MotionMagicJerk = 0; // dddx

        myTalon.getConfigurator().apply(configs);
    }

    // Setting elevator talon to spin to a certain height
    // a, b, x, y control different set heights
    public Command talonSet(double setpoint) {
        return runOnce(() -> 
        {
            voltage = new MotionMagicVoltage(setpoint); // Setpoint = rotations
            myTalon.setControl(voltage); // Rotations

            // SmartDashboard
            SmartDashboard.putNumber("Motor Rotations", myTalon.getPosition().getValueAsDouble()); // Max/min = +-16384 rots
        });
    }

    public double getInitPosition() {
        return myTalon.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        //System.out.println("idk");
        SmartDashboard.putNumber("Rotations", myTalon.getPosition().getValueAsDouble());
    }
}
// Motion magic sysID