package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorScoring extends Command {
    /* Wants of Command
     
     * Claw is open at end of fire to help L4 score
     * Claw is closed until game piece is far enough
     * Rollers eject piece
     
     */

    private final EndEffectorSubsystem endEffector;

    public EndEffectorScoring(EndEffectorSubsystem endEffector) {
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        /* Three states to be accounted for 
         * 1. Neither sensor detects coral
            Command ends
         * 2. Both sensors dectect coral
            Rollers running to score and claw closed
         * 3. Bottom sensor is the only detection
            Rollers running to score and claw open
        */


        if (endEffector.getBothSensors()) {
            //Both see coral
            endEffector.closeClaw();
            endEffector.runRollersUp();
        } else{
            //Bottom sees coral
            endEffector.openClaw();
            endEffector.runRollersUp();
        }
    }

    @Override
    public boolean isFinished() {
        return !endEffector.doWeHaveGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.setClawPower(0);
        endEffector.setRollerPower(0);
    }
}
