package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class AdvancedPID {
    private TalonFX myTalon;

    private MotionMagicVoltage voltage;

    public AdvancedPID() {
        myTalon = new TalonFX(0);

        var talonFXConfigs = new TalonFXConfiguration();
        
        // PDSAV Settings
        var configs = talonFXConfigs.Slot0;
        configs.kP = 0; //
        configs.kD = 0;
        configs.kS = 0;
        configs.kV = 0;
        configs.kA = 0;

        // Motion Magic
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicAcceleration = 0;
        motionMagicConfigs.MotionMagicJerk = 0;

        myTalon.getConfigurator().apply(configs);
    }

    /*public void talonSet(double p) {
        final MotionMagicVoltage voltage = new MotionMagicVoltage(p);
        myTalon.setControl(voltage);
    }*/

    public Command talonSet(double setpoint) {
        return runOnce(() -> 
        {
            voltage = new MotionMagicVoltage(setpoint); // Setpoint = rotations
            myTalon.setControl(voltage);

            // Shuffleboard
            SmartDashboard.putNumber("Motor Rotations", myTalon.getPosition().getValueAsDouble()); // Max/min = +-16384 rots
        });
    }
}
// Motion magic ID