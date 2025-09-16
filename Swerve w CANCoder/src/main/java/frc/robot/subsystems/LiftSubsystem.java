package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase{
    /* Wants of Subsystem

     * PID to control position of elevator
     * Keep in mind chain slop
     * Create zones to limit where tilt can move
     
    */

    //Constants
    private static final int liftMotorID = 20;
    private static final double kP = 0.11;
    private static final double kI = 0.0;
    private static final double kD = 0.012;
    private static final double staticFeedForward = .0;
    private static final double accelFeedForward = .0;
    private static final double rotationsPerInch = 1;
    private static final double startingPointInches = 0;
    private static final double liftClearForRotationStartPoint = 35;
    private static final double liftClearForRotationEndPoint = 55;
    private static final double maxSpeedUp = 1;
    private static final double maxSpeedDown = -0.5;



    private PIDController liftPID = new PIDController(kP, kI, kD);
    private TalonFX liftMotor = new TalonFX(liftMotorID);
    
    public LiftSubsystem() {
        CurrentLimitsConfigs liftCurrentLimit = new CurrentLimitsConfigs();
        liftCurrentLimit.SupplyCurrentLimit = 40.0;
        liftCurrentLimit.SupplyCurrentLowerLimit = 50.0;
        liftCurrentLimit.SupplyCurrentLowerTime = 1.0;
        liftCurrentLimit.SupplyCurrentLimitEnable = true;

        liftCurrentLimit.StatorCurrentLimit = 50.0;
        liftCurrentLimit.StatorCurrentLimitEnable = true;

        TalonFXConfiguration liftMotorConfig = new TalonFXConfiguration();
        liftMotorConfig.withCurrentLimits(liftCurrentLimit);
        liftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        liftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        liftMotorConfig.Voltage.PeakForwardVoltage = 12;
        liftMotorConfig.Voltage.PeakReverseVoltage = -12;
        liftMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.3;
        liftMotor.getConfigurator().apply(liftMotorConfig);
    }


    public void zeroLift() {
        liftMotor.setPosition(0);
    }


    public double getLiftPosition() {
        return (liftMotor.getPosition().getValueAsDouble() * rotationsPerInch) + startingPointInches;
    }

    public boolean liftClearForRotation() {
        return getLiftPosition() >= liftClearForRotationStartPoint && getLiftPosition() <= liftClearForRotationEndPoint;
    }


    public void setPower(double power) {
        power += staticFeedForward;
        power += accelFeedForward * liftMotor.getAcceleration().getValueAsDouble();

        if (power > 0) power = Math.min(power, maxSpeedUp);
        if (power < 0) power = Math.max(power, maxSpeedDown);
        liftMotor.set(power);
    }

    public void setPosition(double setPoint) {
        setPower(liftPID.calculate(getLiftPosition(), setPoint));
    }

    public boolean getHomeLimit() {
        return liftMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    @Override
    public void periodic() {
        smartDashboardOutput();
    }

    public void smartDashboardOutput() {
        SmartDashboard.putNumber("Lift Position", getLiftPosition());
        SmartDashboard.putBoolean("Is Lift Clear for Tilt", liftClearForRotation());
        SmartDashboard.putBoolean("Is Lift At Home", getHomeLimit());
    }
}
