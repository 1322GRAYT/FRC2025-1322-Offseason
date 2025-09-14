package frc.robot.subsystems;

import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftAndTiltSubsystem extends SubsystemBase{
    public LiftSubsystem lift = new LiftSubsystem();
    public TiltSubsystem tilt = new TiltSubsystem();

    public LiftAndTiltSubsystem() {
    }

    
}
