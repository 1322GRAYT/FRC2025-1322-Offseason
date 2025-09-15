package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LiftAndTiltSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.TiltSubsystem;

public class LiftAndTiltJoystick extends Command {
    /* Wants of Command
     
     * Control both the lift and tilt subsystems
     * Set the target point for both
     * Run the PID Controllers for both
     * Prevent the robot from destroying itself
      
     */

    private final LiftSubsystem lift;
    private final TiltSubsystem tilt;
    private CommandXboxController operator;
    
    public LiftAndTiltJoystick(LiftAndTiltSubsystem liftAndTilt, CommandXboxController operator) {
        this.lift = liftAndTilt.lift;
        this.tilt = liftAndTilt.tilt;
        this.operator = operator;
        addRequirements(liftAndTilt, lift, tilt);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //Setting target position
        lift.setPower(-operator.getLeftY() * 0.5);
        tilt.setPower(operator.getRightX() * 0.3);
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
