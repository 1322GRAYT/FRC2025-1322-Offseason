package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    private static final double kP = .001;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double staticFeedForward = .05;
    private static final double accelFeedForward = .05;
    private static final double rotationsPerInch = 1;
    private static final double startingPointInches = 20;
    private static final double liftClearForRotationPoint = 30;
    private static final double maxSpeed = 0.5;


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
        liftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        liftMotorConfig.Voltage.PeakForwardVoltage = 12;
        liftMotorConfig.Voltage.PeakReverseVoltage = -12;
        liftMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        liftMotor.getConfigurator().apply(liftMotorConfig);
    }



    public double getLiftPosition() {
        return (liftMotor.getPosition().getValueAsDouble() * rotationsPerInch) + startingPointInches;
    }

    public boolean liftClearForRotation() {
        return getLiftPosition() >= liftClearForRotationPoint;
    }



    public void setPower(double power) {
        power += staticFeedForward;
        power += accelFeedForward * liftMotor.getAcceleration().getValueAsDouble();

        if (power > maxSpeed) power = maxSpeed;
        liftMotor.set(power);
    }

    public void setPosition(double setPoint) {
        setPower(liftPID.calculate(getLiftPosition(), setPoint));
    }



    @Override
    public void periodic() {
        smartDashboardOutput();
    }

    public void smartDashboardOutput() {
        SmartDashboard.putNumber("Lift Position", getLiftPosition());
        SmartDashboard.putBoolean("Is Lift Clear for Tilt", liftClearForRotation());
    }
}
