package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDefault extends Command {
    /* Wants of Command
     
     * Claw is open during intake
     * Claw is closed once game piece is obtained
     * Rollers keep game piece in end effector
     
     */

    private final ClimberSubsystem climber;
    private Supplier<Boolean> driverYButton;

    public ClimberDefault(ClimberSubsystem climber, Supplier<Boolean> driverYButton) {
        this.climber = climber;
        this.driverYButton = driverYButton;
        addRequirements(climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        /* Three states of climber to be accounted for 
         * 1. Not Climbing
         * 
         * 2. Initaiate Climber
         * 
         * 3. Climb
        */
        
        if (climber.climbMode && driverYButton.get()) {
            if (climber.isGrabClosed()) {
                climber.moveTiltUp();
            } 
            climber.closeGrab();
        } else if (climber.climbMode) {
            climber.moveTiltDown();
            climber.openGrab();
        } else {
            
            climber.moveTiltUp();
            climber.closeGrab();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimbGrabPower(0);
        climber.setClimbTiltPower(0);
    }
}
