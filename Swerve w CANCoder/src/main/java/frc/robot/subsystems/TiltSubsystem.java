package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    private static final double kP = .001;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double staticFeedForward = .05;
    private static final double accelFeedForward = .05;
    private static final double rotationsPerDegree = 1;
    private static final double startingPointDegrees = 20;
    private static final double tiltDangerZoneStartPoint = 150; 
    private static final double tiltDangerZoneEndPoint = 190; 
    private static final double maxSpeed = 0.5;


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



    public double getTiltPosition() {
        return (tiltMotor.getPosition().getValueAsDouble() * rotationsPerDegree) + startingPointDegrees;
    }

    public boolean tiltCrossingOver(double target) {
        boolean crossingFrontToBack = (getTiltPosition() < tiltDangerZoneStartPoint) && (target >= tiltDangerZoneStartPoint);
        boolean crossingBackToFront = (getTiltPosition() > tiltDangerZoneEndPoint) && (target <= tiltDangerZoneStartPoint);
        return crossingFrontToBack || crossingBackToFront;
    }



    public void setPower(double power) {
        power += staticFeedForward;
        power += accelFeedForward * tiltMotor.getAcceleration().getValueAsDouble();

        if (power > maxSpeed) power = maxSpeed;
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
    }
}
