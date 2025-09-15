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
    private double targetLift, targetTilt = 0;
    private Supplier<Integer> targetLevel;
    private Supplier<Boolean> clawClosed;
    
    public LiftAndTiltDefault(LiftAndTiltSubsystem liftAndTilt, Supplier<Integer> targetLevel, Supplier<Boolean> clawClosed) {
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
        //Setting target position
        switch (targetLevel.get()) {
            case 1:
                targetLift = 68;
                targetTilt = 135;
                break;
            case 2:
                targetLift = 60;
                targetTilt = 10;
                break;
            case 3:
                targetLift = 90;
                targetTilt = 10;
                break;
            case 4:
                targetLift = 143;
                targetTilt = 10;
                break;
            default:
                //At intake position until otherwise stated
                targetLift = 60;
                targetTilt = 10;
                break;

        }

        
        //Running PID Controllers
        //Needs to prevent running tilt axis while in danger zone, but still allow intaking
        //Needs to return to the safe zone for tilt axis if its trying to move to other side
        if(tilt.tiltCrossingOver(targetTilt) && !lift.liftClearForRotation() && !clawClosed.get()) {
            //Set lift to safe position
            int tempLift = 75;
            lift.setPosition(tempLift);

            //Stop tilt
            tilt.setPower(0);
        } else {
            //Run the PID Controllers
            //targetLift = 80;
            lift.setPosition(targetLift);
            tilt.setPosition(targetTilt);
        }

        SmartDashboard.putNumber("Lift Target", targetLift);
        SmartDashboard.putNumber("Tilt Target", targetTilt);
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
