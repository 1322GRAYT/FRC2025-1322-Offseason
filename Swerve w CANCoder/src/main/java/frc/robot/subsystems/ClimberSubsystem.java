package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    /* Wants of Subsystem

     * Detect Game piece presence
     * Operate Claw
     * Operate Roller
     
    */

    //Constants
    private static final int climbTiltMotorID = 50;
    private static final int climbGrabMotorID = 51;
    private static final double tiltUpSpeed = 1;
    private static final double tiltDownSpeed = -1;
    private static final double grabCloseSpeed = 0.5;
    private static final double grabOpenSpeed = -0.5;
    private static final double maxSpeedClimbTilt = 1;
    private static final double maxSpeedClimbGrab = 1;

    public boolean climbMode = false;


    private TalonFX climbTiltMotor = new TalonFX(climbTiltMotorID);
    private TalonSRX climbGrabMotor = new TalonSRX(climbGrabMotorID);

    public ClimberSubsystem() {
        CurrentLimitsConfigs climbTiltCurrentLimit = new CurrentLimitsConfigs();
        climbTiltCurrentLimit.SupplyCurrentLimit = 40.0;
        climbTiltCurrentLimit.SupplyCurrentLowerLimit = 50.0;
        climbTiltCurrentLimit.SupplyCurrentLowerTime = 1.0;
        climbTiltCurrentLimit.SupplyCurrentLimitEnable = true;

        climbTiltCurrentLimit.StatorCurrentLimit = 50.0;
        climbTiltCurrentLimit.StatorCurrentLimitEnable = true;

        TalonFXConfiguration climbTiltMotorConfig = new TalonFXConfiguration();
        climbTiltMotorConfig.withCurrentLimits(climbTiltCurrentLimit);
        climbTiltMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbTiltMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climbTiltMotorConfig.Voltage.PeakForwardVoltage = 12;
        climbTiltMotorConfig.Voltage.PeakReverseVoltage = -12;
        climbTiltMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
        climbTiltMotor.getConfigurator().apply(climbTiltMotorConfig);

        zeroPosition();
        climbGrabMotor.setNeutralMode(NeutralMode.Brake);
    }

    
    public void zeroPosition() {
        climbTiltMotor.setPosition(0);
    }
    

    public void setClimbTiltPower(double power) {
        power = Math.min(power, maxSpeedClimbTilt);
        climbTiltMotor.set(power);
    }

    public void setClimbGrabPower(double power) {
        power = Math.min(power, maxSpeedClimbGrab);
        climbGrabMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public boolean isTiltDown() {
        return climbTiltMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public boolean isTiltUp() {
        return climbTiltMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean isGrabOpen() {
        return climbGrabMotor.isRevLimitSwitchClosed()==1;
    }

    public boolean isGrabClosed() {
        return climbGrabMotor.isFwdLimitSwitchClosed()==1;
    }



    public void moveTiltDown() {
        if (!isTiltDown()) {
            setClimbTiltPower(tiltDownSpeed);
        } else {
            setClimbTiltPower(0);
        }
    }

    public void moveTiltUp() {
        if (!isTiltUp()) {
            setClimbTiltPower(tiltUpSpeed);
        } else {
            setClimbTiltPower(0);
        }
    }

    public void moveTiltUpToClimb() {
        if (climbTiltMotor.getPosition().getValueAsDouble() < -15) {
            setClimbTiltPower(tiltUpSpeed);
        } else {
            setClimbTiltPower(0);
        }
    }

    public void closeGrab() {
        if (!isGrabClosed()) {
            setClimbGrabPower(grabCloseSpeed);
        } else {
            setClimbGrabPower(0);
        }
    }

    public void openGrab() {
        if (!isGrabOpen()) {
            setClimbGrabPower(grabOpenSpeed);
        } else {
            setClimbGrabPower(0);
        }
    }



    @Override
    public void periodic() {
        smartDashboardOutput();
    }

    public void smartDashboardOutput() {
        SmartDashboard.putBoolean("Is Climber Open", isGrabOpen());
        SmartDashboard.putBoolean("Is Climber Closed", isGrabOpen());
        SmartDashboard.putBoolean("Is Climber Down", isTiltDown());
        SmartDashboard.putNumber("Climb Tilt Position", climbTiltMotor.getPosition().getValueAsDouble());
    }

}
