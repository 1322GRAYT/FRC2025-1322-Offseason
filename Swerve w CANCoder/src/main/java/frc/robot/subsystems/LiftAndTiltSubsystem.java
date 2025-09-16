package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftAndTiltSubsystem extends SubsystemBase{
    public LiftSubsystem lift = new LiftSubsystem();
    public TiltSubsystem tilt = new TiltSubsystem();
    public boolean homed = false;

    public LiftAndTiltSubsystem() {
        lift.zeroLift();
        tilt.zeroTilt();
    }

    public void haveWeHomed(boolean homed) {
        lift.haveWeHomed(homed);
        tilt.haveWeHomed(homed);
        this.homed = homed;
    }

    public boolean liftAndTiltAtIntake() {
        return ((tilt.getTiltPosition() > TiltSubsystem.tiltDangerZoneEndPoint) && lift.getLiftPosition() < 35);
    }

}
