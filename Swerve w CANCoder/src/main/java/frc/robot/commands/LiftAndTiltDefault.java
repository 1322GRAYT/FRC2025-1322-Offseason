package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftAndTiltSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.TiltSubsystem;

public class LiftAndTiltDefault extends Command {
    /* Wants of Command
     
     * Control both the lift and tilt subsystems
     * Set the target point for both
     * Run the PID Controllers for both
     * Prevent the robot from destroying itself
      
     */

    private final LiftSubsystem lift;
    private final TiltSubsystem tilt;
    private final LiftAndTiltSubsystem liftAndTilt;
    private double targetLift, targetTilt = 0;
    private Supplier<Integer> targetLevel;
    private Supplier<Boolean> clawClosed;
    
    public LiftAndTiltDefault(LiftAndTiltSubsystem liftAndTilt, Supplier<Integer> targetLevel, Supplier<Boolean> clawClosed) {
        this.liftAndTilt = liftAndTilt;
        this.lift = liftAndTilt.lift;
        this.tilt = liftAndTilt.tilt;
        this.targetLevel = targetLevel;
        this.clawClosed = clawClosed;
        addRequirements(liftAndTilt, lift, tilt);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        if (liftAndTilt.homed) {

            
            //Setting target position
            switch (targetLevel.get()) {
                case 1:
                    targetLift = 40;
                    targetTilt = 40;
                    break;
                case 2:
                    targetLift = 23;
                    targetTilt = 0;
                    break;
                case 3:
                    targetLift = 60;
                    targetTilt = 0;
                    break;
                case 4:
                    targetLift = 113;
                    targetTilt = 0;
                    break;
                default:
                    //At intake position until otherwise stated
                    targetLift = 27;
                    targetTilt = 135;
                    break;

            }

            
            //Running PID Controllers
            //Needs to prevent running tilt axis while in danger zone, but still allow intaking
            //Needs to return to the safe zone for tilt axis if its trying to move to other side
            //(tilt.tiltCrossingOver(targetTilt) && !lift.liftClearForRotation()) || (!clawClosed.get() && tilt.tiltInDangerZone())
            if((tilt.tiltCrossingOver(targetTilt) && !lift.liftClearForRotation()) || (!clawClosed.get() && tilt.tiltInDangerZone())) {
                //Set lift to safe position
                int tempLift = 45;
                lift.setPosition(tempLift);

                //Stop tilt
                tilt.setPower(0);
            } else if (tilt.tiltCrossingOver(targetTilt) || tilt.tiltInDangerZone()) {
                int tempLift = 45;
                lift.setPosition(tempLift);

                tilt.setPosition(targetTilt);

            } else {

                //Run the PID Controllers
                //targetLift = 80;
                lift.setPosition(targetLift);
                tilt.setPosition(targetTilt);
            }

            SmartDashboard.putNumber("Lift Target", targetLift);
            SmartDashboard.putNumber("Tilt Target", targetTilt);
        } else {
           // if(lift.)
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        lift.setPower(0);
        tilt.setPower(0);
    }
}
