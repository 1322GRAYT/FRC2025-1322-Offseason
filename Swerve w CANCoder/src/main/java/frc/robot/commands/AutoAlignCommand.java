package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignCommand extends Command{
    /* Wants for Command
     * 
     * Trapezoidal Profile to control field centric facing angle controller
     * 
     */

    private final CommandSwerveDrivetrain drive;

    private SwerveRequest swerveRequest;

    private SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withRotationalDeadband(0.05);

    private final ProfiledPIDController pid = new ProfiledPIDController(
        1.5, 0, 0, 
        new TrapezoidProfile.Constraints(1, 1)
    );    
    private PhoenixPIDController turnPID = new PhoenixPIDController(7, 0, 0);


    public AutoAlignCommand(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveFacingAngle.HeadingController = turnPID;
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        Pose2d targetPose = drive.closestBranch();

        double[] printPose = {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("target Pose", printPose);

        double distanceAwayX = currentPose.getX() - targetPose.getX();
        double distanceAwayY = currentPose.getY() - targetPose.getY();
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        double output = pid.calculate(distanceAway, 0);


        double newRotation = targetPose.getRotation().getRadians();
        double newForward = output * Math.cos(angleOfDistance);
        double newStrafe = output * Math.sin(angleOfDistance);
        
        // Offsetting for red and blue driver perspective
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            newForward *= -1;
            newStrafe *= -1;
            newRotation -= Math.PI;
        } 

        swerveRequest = driveFacingAngle
            .withTargetDirection(Rotation2d.fromRadians(newRotation))
            .withVelocityX(newForward)
            .withVelocityY(newStrafe);

        drive.setControl(swerveRequest);

        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
