package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase{
    /* Wants of Subsystem

     * Detect Game piece presence
     * Operate Claw
     * Operate Roller
     
    */

    //Constants
    private static final int clawMotorID = 40;
    private static final int rollerMotorID = 41;
    private static final int topSensorID = 0;
    private static final int bottomSensorID = 1;
    private static final double closeClawSpeed = -0.5;
    private static final double openClawSpeed = 0.5;
    private static final double rollerUpSpeed = 0.5;
    private static final double rollerDownSpeed = -0.5;
    private static final double maxSpeedClaw = 1;
    private static final double maxSpeedRoller = 1;


    private TalonFX clawMotor = new TalonFX(clawMotorID);
    private TalonFX rollerMotor = new TalonFX(rollerMotorID);

    private DigitalInput topSensor = new DigitalInput(topSensorID);
    private DigitalInput bottomSensor = new DigitalInput(bottomSensorID);

    public EndEffectorSubsystem() {
        CurrentLimitsConfigs clawCurrentLimit = new CurrentLimitsConfigs();
        clawCurrentLimit.SupplyCurrentLimit = 40.0;
        clawCurrentLimit.SupplyCurrentLowerLimit = 50.0;
        clawCurrentLimit.SupplyCurrentLowerTime = 1.0;
        clawCurrentLimit.SupplyCurrentLimitEnable = true;

        clawCurrentLimit.StatorCurrentLimit = 50.0;
        clawCurrentLimit.StatorCurrentLimitEnable = true;

        CurrentLimitsConfigs rollerCurrentLimit = new CurrentLimitsConfigs();
        rollerCurrentLimit.SupplyCurrentLimit = 40.0;
        rollerCurrentLimit.SupplyCurrentLowerLimit = 50.0;
        rollerCurrentLimit.SupplyCurrentLowerTime = 1.0;
        rollerCurrentLimit.SupplyCurrentLimitEnable = true;

        rollerCurrentLimit.StatorCurrentLimit = 50.0;
        rollerCurrentLimit.StatorCurrentLimitEnable = true;

        TalonFXConfiguration clawMotorConfig = new TalonFXConfiguration();
        clawMotorConfig.withCurrentLimits(clawCurrentLimit);
        clawMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        clawMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        clawMotorConfig.Voltage.PeakForwardVoltage = 12;
        clawMotorConfig.Voltage.PeakReverseVoltage = -12;
        clawMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        clawMotor.getConfigurator().apply(clawMotorConfig);

        TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.withCurrentLimits(rollerCurrentLimit);
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.Voltage.PeakForwardVoltage = 12;
        rollerMotorConfig.Voltage.PeakReverseVoltage = -12;
        rollerMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }


    public boolean isClawClosed() {
        return clawMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean isClawOpen() {
        return clawMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean getTopSensor() {
        return topSensor.get();
    }

    public boolean getBottomSensor() {
        return bottomSensor.get();
    }

    public boolean doWeHaveGamePiece() {
        return getTopSensor() || getBottomSensor();
    }

    public boolean getBothSensors() {
        return getTopSensor() && getBottomSensor();
    }



    public void setClawPower(double power) {
        power = Math.min(power, maxSpeedClaw);
        clawMotor.set(power);
    }

    public void setRollerPower(double power) {
        power = Math.min(power, maxSpeedRoller);
        clawMotor.set(power);
    }

    public void closeClaw() {
        if (!isClawClosed()) {
            setClawPower(closeClawSpeed);
        } else {
            setClawPower(0);
        }
    }

    public void openClaw() {
        if (!isClawOpen()) {
            setClawPower(openClawSpeed);
        } else {
            setClawPower(0);
        }
    }

    public void runRollersUp() {
        setRollerPower(rollerUpSpeed);
    }

    public void runRollersDown() {
        setRollerPower(rollerDownSpeed);
    }




    @Override
    public void periodic() {
        smartDashboardOutput();
    }

    public void smartDashboardOutput() {
        SmartDashboard.putBoolean("Top Sensor", getTopSensor());
        SmartDashboard.putBoolean("Bottom Sensor", getBottomSensor());
        SmartDashboard.putBoolean("Claw Opened", isClawOpen());
        SmartDashboard.putBoolean("Claw Closed", isClawClosed());
    }

}
