package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }


    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        smartDashboardOutput();
        if (SmartDashboard.getBoolean("Update Robot Angle Using Vision?", true)) {
            updateRobotPoseMT1();
        } else {
            updateRobotPoseMT2();
        }
    }

    public void smartDashboardOutput() {
        double[] printPose = {getPose().getX(), getPose().getY(), getPose().getRotation().getRadians()};
        SmartDashboard.putNumberArray("Robot Pose", printPose);
    }




    public Pose2d getPose() {
        return this.getState().Pose;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void updateRobotPoseMT1() {
        boolean updateVision = true;

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
            updateVision = false;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
            updateVision = false;
        }
      }
      if(mt1.tagCount == 0)
      {
        updateVision = false;
      }

      if(updateVision)
      {
        //SmartDashboard.putNumber("Hello", mt1.pose.getX());
        setVisionMeasurementStdDevs(VecBuilder.fill(.05,.05,Math.toRadians(1)));
        addVisionMeasurement(
            mt1.pose,
            Utils.fpgaToCurrentTime(mt1.timestampSeconds));
      }    
    }

    public void updateRobotPoseMT2() {
        boolean updateVision = true;
        LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        if(Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 360)
        {
            updateVision = false;
        }
        if(mt2.tagCount == 0)
        {
            updateVision = false;
        }
        if(updateVision)
        {
        setVisionMeasurementStdDevs(VecBuilder.fill(.05,.05,9999999));
        addVisionMeasurement(
            mt2.pose,
            Utils.fpgaToCurrentTime(mt2.timestampSeconds));
        }
    }






    ////////////////////////////////////// Auto align ////////////////////////////////////////////////////////////////

    // Red Reef
    private static final Pose2d TAG_SIX = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-60));
    private static final Pose2d TAG_SEVEN = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0));
    private static final Pose2d TAG_EIGHT = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60));
    private static final Pose2d TAG_NINE = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120));
    private static final Pose2d TAG_TEN = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180));
    private static final Pose2d TAG_ELEVEN = new Pose2d(Units.inchesToMeters(497.77 ), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-120));
    
    // Blue Reef
    private static final Pose2d TAG_SEVENTEEN = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-120));
    private static final Pose2d TAG_EIGHTEEN = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180));
    private static final Pose2d TAG_NINETEEN = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120));
    private static final Pose2d TAG_TWENTY = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60));
    private static final Pose2d TAG_TWENTY_ONE = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0));
    private static final Pose2d TAG_TWENTY_TWO = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-60));

    //Offsets to branch
    private static final double tagToLeftBranch = Units.inchesToMeters(-6);
    private static final double tagToRightBranch = Units.inchesToMeters(6);
    private static final double targetDistanceFromTag = Units.inchesToMeters(15);

    private static final double leftBoundOfReef = (TAG_SIX.getY() + TAG_SEVEN.getY()) / 2;
    private static final double rightBoundOfReef = (TAG_EIGHT.getY() + TAG_SEVEN.getY()) / 2;

    private boolean isLeftBranch = true;

    public void setLeftOrRightBranch(boolean isLeftBranch) {
        this.isLeftBranch = isLeftBranch;
    }

    public Pose2d closestBranch() {
        Pose2d outputPose;
        //Varible is 0, 1, or 2, for left, middle, or right side of field
        int yPosOnField;
        if (getPose().getY() < leftBoundOfReef) yPosOnField = 0;
        else if (getPose().getY() > rightBoundOfReef) yPosOnField = 2;
        else yPosOnField = 1;


        //Sets our output pose to the closest side of the reef
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            switch (yPosOnField) {
                case 0:
                    if (getPose().getX() > (TAG_SEVENTEEN.getX() + TAG_TWENTY_TWO.getX()) / 2) outputPose = TAG_TWENTY_TWO;
                    else outputPose = TAG_SEVENTEEN;
                    break;
                case 1:
                    if (getPose().getX() > (TAG_EIGHTEEN.getX() + TAG_TWENTY_ONE.getX()) / 2) outputPose = TAG_TWENTY_ONE;
                    else outputPose = TAG_EIGHTEEN;
                    break;
                default:
                    if (getPose().getX() > (TAG_NINETEEN.getX() + TAG_TWENTY.getX()) / 2) outputPose = TAG_TWENTY;
                    else outputPose = TAG_NINETEEN;
                    break;
            }
        } else {
            switch (yPosOnField) {
                case 0:
                    if (getPose().getX() > (TAG_SIX.getX() + TAG_ELEVEN.getX()) / 2) outputPose = TAG_SIX;
                    else outputPose = TAG_ELEVEN;
                    break;
                case 1:
                    if (getPose().getX() > (TAG_SEVEN.getX() + TAG_TEN.getX()) / 2) outputPose = TAG_SEVEN;
                    else outputPose = TAG_TEN;
                    break;
                default:
                    if (getPose().getX() > (TAG_EIGHT.getX() + TAG_NINE.getX()) / 2) outputPose = TAG_EIGHT;
                    else outputPose = TAG_NINE;
                    break;
            }
        }

        outputPose.transformBy(new Transform2d(
            isLeftBranch ? tagToLeftBranch : tagToRightBranch, //Left or Right
            targetDistanceFromTag, //Offset back from reef
            outputPose.getRotation()
        ));

        return outputPose;
    }
}
