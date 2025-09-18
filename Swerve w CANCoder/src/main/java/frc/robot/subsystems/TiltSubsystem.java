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

public class TiltSubsystem extends SubsystemBase{
    /* Wants of Subsystem

     * PID to control position of tilt
     * Create zones to limit where lift can move
     
    */

    //Constants
    private static final int tiltMotorID = 30;
    private static final double kP = .1;
    private static final double kI = 0;
    private static final double kD = 0.001;
    private static final double staticFeedForward = .0;
    private static final double accelFeedForward = .0;
    private static final double rotationsPerDegree = 1;
    private static final double startingPointDegrees = 0;
    public static final double tiltDangerZoneStartPoint = 60; 
    public static final double tiltDangerZoneEndPoint = 100; 
    private static final double maxSpeed = 1;

    private boolean crossingOver = false;

    private PIDController tilt = new PIDController(kP, kI, kD);
    private TalonFX tiltMotor = new TalonFX(tiltMotorID);
    
    public TiltSubsystem() {
        CurrentLimitsConfigs tiltCurrentLimit = new CurrentLimitsConfigs();
        tiltCurrentLimit.SupplyCurrentLimit = 40.0;
        tiltCurrentLimit.SupplyCurrentLowerLimit = 50.0;
        tiltCurrentLimit.SupplyCurrentLowerTime = 1.0;
        tiltCurrentLimit.SupplyCurrentLimitEnable = true;

        tiltCurrentLimit.StatorCurrentLimit = 50.0;
        tiltCurrentLimit.StatorCurrentLimitEnable = true;

        TalonFXConfiguration tiltMotorConfig = new TalonFXConfiguration();
        tiltMotorConfig.withCurrentLimits(tiltCurrentLimit);
        tiltMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        tiltMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        tiltMotorConfig.Voltage.PeakForwardVoltage = 12;
        tiltMotorConfig.Voltage.PeakReverseVoltage = -12;
        tiltMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        tiltMotor.getConfigurator().apply(tiltMotorConfig);
    }

    public void zeroTilt() {
        tiltMotor.setPosition(0);
    }

    public double getTiltPosition() {
        return (tiltMotor.getPosition().getValueAsDouble() * rotationsPerDegree) + startingPointDegrees;
    }

    public boolean tiltCrossingOver(double target) {
        boolean crossingFrontToBack = (getTiltPosition() < tiltDangerZoneEndPoint) && (target >= tiltDangerZoneEndPoint);
        boolean crossingBackToFront = (getTiltPosition() > tiltDangerZoneStartPoint) && (target <= tiltDangerZoneStartPoint);
        crossingOver = crossingFrontToBack || crossingBackToFront;
        return crossingOver || tiltInDangerZone();
    }

    public boolean tiltCrossingOver() {
        return crossingOver || tiltInDangerZone();
    }

    public boolean tiltInDangerZone() {
        return ((getTiltPosition() > tiltDangerZoneStartPoint) && (getTiltPosition() < tiltDangerZoneEndPoint)) || (getTiltPosition() < -5);
    }

    public boolean getHomeLimit() {
        return tiltMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }


    public void setPower(double power) {
        power += staticFeedForward;
        power += accelFeedForward * tiltMotor.getAcceleration().getValueAsDouble();

        power = Math.min(power, maxSpeed);
        tiltMotor.set(power);
    }

    public void setPosition(double setPoint) {
        setPower(tilt.calculate(getTiltPosition(), setPoint));
    }



    @Override
    public void periodic() {
        smartDashboardOutput();
    }

    public void smartDashboardOutput() {
        SmartDashboard.putNumber("Tilt Position", getTiltPosition());
        SmartDashboard.putBoolean("Get Home Sensor", getHomeLimit());
    }
}
