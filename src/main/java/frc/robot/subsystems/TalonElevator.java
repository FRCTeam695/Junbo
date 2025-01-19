package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.Volts; // Static so that everything is there (No need to write Units.)

public class TalonElevator extends SubsystemBase{ // EXTENDS SUBSYSTEMBASE!!!!!!!!!
    private TalonFX myTalon;
    private MotionMagicVoltage m_request;
    private VoltageOut m_voltReq;

    // 50Hz NetworkTable variables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable elevatorTable = inst.getTable("Elevator");

    private final DoublePublisher motorRotPub = elevatorTable.getDoubleTopic("Motor rotations").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher closedLoopPub = elevatorTable.getDoubleTopic("Closed Loop Output").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher FFPub = elevatorTable.getDoubleTopic("Feed Forward").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher velocityPub = elevatorTable.getDoubleTopic("Velocity").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher velocityTargetPub = elevatorTable.getDoubleTopic("Velocity Target").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher rotationsTargetPub = elevatorTable.getDoubleTopic("Position Target").publish(PubSubOption.periodic(0.02));

    // Constructor
    public TalonElevator() {
        myTalon = new TalonFX(50); // Falcon 500

        var talonFXConfigs = new TalonFXConfiguration(); // All paramater configs
        m_request = new MotionMagicVoltage(0); // Trapezoid config

        m_voltReq = new VoltageOut(0);

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
        slot0Configs.kS = 0.07; // Friction 0.07 volt
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        slot0Configs.kV = 0.13; // volt/rps
        slot0Configs.kA = 0.008; // volt/rps/s //0.008
        slot0Configs.kP = 3.5; // volt/(r*s) //3.5
        slot0Configs.kD = 0.1; // volt/rps //0.1

        // Motion Magic (Trapezoid speed control)
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; 
        motionMagicConfigs.MotionMagicAcceleration = 150; 
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
        SmartDashboard.putNumber("Motor Rotations", myTalon.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Closed Loop Output", myTalon.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("FF Output", myTalon.getClosedLoopFeedForward().getValueAsDouble());
        SmartDashboard.putNumber("Velocity", myTalon.getVelocity().getValueAsDouble());
        
        motorRotPub.set(myTalon.getPosition(true).getValueAsDouble());
        closedLoopPub.set(myTalon.getClosedLoopProportionalOutput(true).getValueAsDouble());
        FFPub.set(myTalon.getClosedLoopFeedForward(true).getValueAsDouble());
        velocityPub.set(myTalon.getVelocity(true).getValueAsDouble());
        velocityTargetPub.set(myTalon.getClosedLoopReferenceSlope(true).getValueAsDouble());
        rotationsTargetPub.set(myTalon.getClosedLoopReference(true).getValueAsDouble());
    }
}

// SysID
/*private SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
        null, // Default ramp rate (1V/s)
        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
        null, // Default timeout (10s)

        (state) -> SignalLogger.writeString("State", state.toString())
    ), 
    new SysIdRoutine.Mechanism(
        (volts) -> myTalon.setControl(m_voltReq.withOutput(volts.in(Volts))),
        null, // Left null when using a signal logger
        this
    )
);

public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
}
 
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
}*/