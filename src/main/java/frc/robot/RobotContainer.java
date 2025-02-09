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
import frc.robot.Constants.KitBot25Constants;
import frc.robot.Constants.KitBot24Constants;
import frc.robot.Constants.Amper24Constants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitBot25;
import frc.robot.subsystems.KitBot24;
import frc.robot.subsystems.Amper24;

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
    public final KitBot25 kitbot25 = new KitBot25();
    public final KitBot24 kitbot24 = new KitBot24();
    public final Amper24 amper24 = new Amper24();

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
                drive.withVelocityX(-driver_controller.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_controller.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(driver_controller.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with X (right)
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

        // KitBot 2025 first piece eject
        operator_controller.b()
        .whileTrue(kitbot25.runRoller(kitbot25, () -> KitBot25Constants.ROLLER_FIRST_EJECT_VALUE, () -> 0)).onFalse(
            Commands.runOnce(() -> {
                KitBot25.rollerMotor.stopMotor();
            }, kitbot25)
        );

        // KitBot 2025 stacked piece eject
        operator_controller.y()
        .whileTrue(kitbot25.runRoller(kitbot25, () -> KitBot25Constants.ROLLER_STACKED_EJECT_VALUE, () -> 0)).onFalse(
            Commands.runOnce(() -> {
                KitBot25.rollerMotor.stopMotor();
            }, kitbot25)
        );

        // KitBot 2025 re-align
        operator_controller.x()
        .whileTrue(kitbot25.runRoller(kitbot25, () -> -KitBot25Constants.ROLLER_FIRST_EJECT_VALUE, () -> 0)).onFalse(
            Commands.runOnce(() -> {
                KitBot25.rollerMotor.stopMotor();
            }, kitbot25)
        );

        // KitBot 2024 intake
        operator_controller.leftBumper().whileTrue(Commands.run(() -> {
            KitBot24.feeder_motor.set(KitBot24Constants.kIntakeFeederSpeed);
            KitBot24.launcher_motor.set(KitBot24Constants.kIntakeLauncherSpeed);
        }, kitbot24));

        // KitBot 2024 shoot
        operator_controller.rightBumper().whileTrue(Commands.sequence(
            Commands.runOnce(() -> KitBot24.launcher_motor.set(KitBot24Constants.kShootLauncherSpeed), kitbot24),
            Commands.waitSeconds(KitBot24Constants.kFeederDelay),
            Commands.run(() -> {
                KitBot24.launcher_motor.set(KitBot24Constants.kShootLauncherSpeed);
                KitBot24.feeder_motor.set(KitBot24Constants.kShootFeederSpeed);
            }, kitbot24))
            ).onFalse(
                Commands.runOnce(() -> {
                    KitBot24.feeder_motor.stopMotor();
                    KitBot24.launcher_motor.stopMotor();
                }, kitbot24)
            );

        // Amper 2024 intake
        operator_controller.a().whileTrue(Commands.run(() -> 
            Amper24.amper_motor.setVoltage(Amper24Constants.kAmperIntakeSpeed * 12), amper24)
            ).onFalse(
                Commands.runOnce(() -> 
                Amper24.amper_motor.stopMotor(), amper24)
            );

        // Amper 2024 score
        operator_controller.y().whileTrue(Commands.run(() -> 
            Amper24.amper_motor.setVoltage(Amper24Constants.kAmperScoreSpeed * 12), amper24)
            ).onFalse(
                Commands.runOnce(() -> 
                Amper24.amper_motor.stopMotor(), amper24)
            );

        // Amper 2024 hold position
        operator_controller.x().onTrue(Commands.runOnce(() -> {
            if (!amper24.isHoldPositionActive) {
                Amper24.amper_motor.setVoltage(Amper24Constants.kAmperHoldPositionSpeed * 12);
                amper24.isHoldPositionActive = true;
            } else {
                Amper24.amper_motor.set(0);
                amper24.isHoldPositionActive = false;
            }
        }, amper24));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void disabledInit() {
        KitBot25.rollerMotor.stopMotor();
        KitBot24.feeder_motor.stopMotor();
        KitBot24.launcher_motor.stopMotor();
        Amper24.amper_motor.stopMotor();
    }
}
