package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorDefault extends Command {
    /* Wants of Command
     
     * Claw is open during intake
     * Claw is closed once game piece is obtained
     * Rollers keep game piece in end effector
     
     */

    private final EndEffectorSubsystem endEffector;
    private Supplier<Boolean> tiltCrossingOver, clawAtIntake;

    public EndEffectorDefault(EndEffectorSubsystem endEffector, Supplier<Boolean> tiltCrossingOver, Supplier<Boolean> clawAtIntake) {
        this.endEffector = endEffector;
        this.tiltCrossingOver = tiltCrossingOver;
        this.clawAtIntake = clawAtIntake;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        /* Three states of claw to be accounted for 
         * 1. If arm crossing over, close claw
         * 
         * 2. intaking and no game piece, open claw
         * 
         * 3. have game piece, close claw
        */

        /* Three states of roller to be accounted for 
         * 1. both sensors same, stop rollers
         * 
         * 2. top sensor only, roll down
         * 
         * 3. bottom sensor only, roll up
        */

        if (!endEffector.doWeHaveGamePiece() && !tiltCrossingOver.get()) {
            endEffector.openClaw();
        } else {
            endEffector.closeClaw();
        }


        if (endEffector.getBothSensors()) {
            endEffector.setRollerPower(0);
        } else if (endEffector.getTopSensor() || (clawAtIntake.get() && !endEffector.getBottomSensor())) {
            endEffector.runRollersDown();
        } else if (endEffector.getBottomSensor()) {
            endEffector.runRollersUp();
        } else {
            endEffector.setRollerPower(0);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setClawPower(0);
        endEffector.setRollerPower(0);
    }
}
