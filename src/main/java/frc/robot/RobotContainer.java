// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.KitBotConstants;
import frc.robot.Constants.AmperConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitBot;
import frc.robot.subsystems.Amper;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 20% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver_controller = new CommandXboxController(0);
    private final CommandXboxController operator_controller = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final KitBot kitbot = new KitBot();
    public final Amper amper = new Amper();

    private final SendableChooser<Command> autoChooser;
    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    public RobotContainer() {
        // Create auto chooser and put it on the Auto tab in Shuffleboard
        autoChooser = AutoBuilder.buildAutoChooser("Example Auto");
        autoTab.add("Auto Mode", autoChooser)
            .withSize(2, 1)
            .withPosition(0, 0);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver_controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver_controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver_controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_controller.getLeftY(), -driver_controller.getLeftX()))
        ));

        driver_controller.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver_controller.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver_controller.back().and(driver_controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver_controller.back().and(driver_controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver_controller.start().and(driver_controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver_controller.start().and(driver_controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driver_controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // KitBot bindings
        operator_controller.leftBumper().whileTrue(Commands.run(() -> {
            KitBot.feeder_motor.set(KitBotConstants.kIntakeFeederSpeed);
            KitBot.launcher_motor.set(KitBotConstants.kIntakeLauncherSpeed);
        }, kitbot));

        operator_controller.rightBumper().whileTrue(Commands.sequence(
            Commands.runOnce(() -> KitBot.launcher_motor.set(KitBotConstants.kShootLauncherSpeed), kitbot),
            Commands.waitSeconds(KitBotConstants.kFeederDelay),
            Commands.run(() -> {
                KitBot.launcher_motor.set(KitBotConstants.kShootLauncherSpeed);
                KitBot.feeder_motor.set(KitBotConstants.kShootFeederSpeed);
            }, kitbot)
        ));

        // Amper bindings
        operator_controller.a().whileTrue(Commands.run(() -> 
            Amper.amper_motor.setVoltage(AmperConstants.kAmperIntakeSpeed * 12), amper));

        operator_controller.y().whileTrue(Commands.run(() -> 
            Amper.amper_motor.setVoltage(AmperConstants.kAmperScoreSpeed * 12), amper));

        operator_controller.x().onTrue(Commands.runOnce(() -> {
            if (!amper.isHoldPositionActive) {
                Amper.amper_motor.setVoltage(AmperConstants.kAmperHoldPositionSpeed * 12);
                amper.isHoldPositionActive = true;
            } else {
                Amper.amper_motor.set(0);
                amper.isHoldPositionActive = false;
            }
        }, amper));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void disabledInit() {
        Amper.amper_motor.stopMotor();
        KitBot.feeder_motor.stopMotor();
        KitBot.launcher_motor.stopMotor();
    }
}
