package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorDefault extends Command {
    /* Wants of Command
     
     * Claw is open during intake
     * Claw is closed once game piece is obtained
     * Rollers keep game piece in end effector
     
     */

    private final EndEffectorSubsystem endEffector;
    private Supplier<Boolean> tiltLimit;

    public EndEffectorDefault(EndEffectorSubsystem endEffector, Supplier<Boolean> tiltLimit) {
        this.endEffector = endEffector;
        this.tiltLimit = tiltLimit;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        /* Four states to be accounted for 
         * 1. Neither sensor detects coral
            Rollers need to be off and claw open
         * 2. Both sensors dectect coral
            Rollers off and claw closed
         * 3. Top sensor is the only detection
            Rollers running toward bottom and claw open
         * 4. Bottom sensor is the only detection
            Rollers running toward top and claw closed
        */

        // if (!endEffector.doWeHaveGamePiece() && tiltLimit.get()) {
        //     endEffector.openClaw();
        // } else {
            endEffector.closeClaw();
        //}

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
