package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftAndTiltSubsystem extends SubsystemBase{
    public LiftSubsystem lift = new LiftSubsystem();
    public TiltSubsystem tilt = new TiltSubsystem();

    public LiftAndTiltSubsystem() {
    }

    
}
