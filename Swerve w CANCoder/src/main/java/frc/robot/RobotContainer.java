// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.EndEffectorDefault;
import frc.robot.commands.EndEffectorScoring;
import frc.robot.commands.LiftAndTiltDefault;
import frc.robot.commands.LiftAndTiltJoystick;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LiftAndTiltSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private int targetLevel = 0;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LiftAndTiltSubsystem liftAndTilt = new LiftAndTiltSubsystem();
    public final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();

    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        // NamedCommands.registerCommand("Level 1", new InstantCommand(() -> setTargetLevel(1)));
        // NamedCommands.registerCommand("Level 2", new InstantCommand(() -> setTargetLevel(2)));
        // NamedCommands.registerCommand("Level 3", new InstantCommand(() -> setTargetLevel(3)));
        // NamedCommands.registerCommand("Level 4", new InstantCommand(() -> setTargetLevel(4)));
        // NamedCommands.registerCommand("Score", new EndEffectorScoring(endEffector));

        // // Build an auto chooser. This will use Commands.none() as the default option.
        // autoChooser = AutoBuilder.buildAutoChooser();

        // // Another option that allows you to specify the default auto by its name
        // // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(scaledDeadband(-driverJoystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(scaledDeadband(-driverJoystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(scaledDeadband(-driverJoystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        liftAndTilt.setDefaultCommand(new LiftAndTiltDefault(liftAndTilt, () -> getTargetLevel(), () -> endEffector.isClawClosed()));
        // liftAndTilt.setDefaultCommand(new LiftAndTiltJoystick(liftAndTilt, operatorJoystick));
        
        endEffector.setDefaultCommand(new EndEffectorDefault(endEffector, () -> liftAndTilt.tilt.tiltCrossingOver()));

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));


        // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverJoystick.rightTrigger().whileTrue(new EndEffectorScoring(endEffector));

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public static double scaledDeadband(double input) {
        //deadband of .1
        if (Math.abs(input) < .1) return 0;

        return Math.copySign((Math.abs(input) - .1 ) / (1 - .1), input);
        
    }

    public int getTargetLevel() {
        if (operatorJoystick.x().getAsBoolean()) targetLevel = 1;
        else if (operatorJoystick.a().getAsBoolean()) targetLevel = 2;
        else if (operatorJoystick.b().getAsBoolean()) targetLevel = 3;
        else if (operatorJoystick.y().getAsBoolean()) targetLevel = 4;
        else if (operatorJoystick.povDown().getAsBoolean() || !endEffector.doWeHaveGamePiece()) targetLevel = 0;

        return targetLevel;
    }

    public void setTargetLevel(int targetLevel) {
        this.targetLevel = targetLevel;
    }

    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
    // }
}
