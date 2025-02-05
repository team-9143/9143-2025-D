// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlLowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlLow;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver_controller = new CommandXboxController(0);
    private final CommandXboxController operator_controller = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevator = new Elevator();
    private final AlLow allow = new AlLow();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Example Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

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

        // Elevator preset heights
        operator_controller.povDown().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.CORAL_L1_HEIGHT), elevator));
        operator_controller.povLeft().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.CORAL_L2_HEIGHT), elevator));
        operator_controller.povRight().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.CORAL_L3_HEIGHT), elevator));
        operator_controller.povUp().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.CORAL_L4_HEIGHT), elevator));

        // Elevator retraction to base height
        operator_controller.x().onTrue(Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.BASE_HEIGHT), elevator));

        // Elevator encoder reset
        operator_controller.a().onTrue(Commands.runOnce(() -> elevator.resetEncoders(), elevator).ignoringDisable(true));

        // Elevator manual control
        elevator.setDefaultCommand(Commands.run(() -> {
            double speed = -operator_controller.getLeftY(); // Use left stick Y for manual control
            elevator.manualControl(speed);
        }, elevator));

        // AlLow extension (with rollers activated)
        operator_controller.rightBumper().whileTrue(Commands.run(() -> {
            elevator.setPosition(AlLowConstants.INTAKE_ANGLE); // Extend to intaking angle
            allow.setRollerSpeed(-1.0); // Spin rollers inward
        }, elevator, allow));

        // AlLow retraction (with rollers stopped)
        operator_controller.leftBumper().onTrue(Commands.runOnce(() -> {
            elevator.setPosition(AlLowConstants.BASE_ANGLE); // Retract to base angle
            allow.stopRoller(); // Stop rollers
        }, elevator, allow));

        // AlLow ejection
        operator_controller.b().whileTrue(Commands.run(() -> allow.setRollerSpeed(1.0), allow));

        // AlLow pivot encoder reset
        operator_controller.y().onTrue(Commands.runOnce(() -> allow.resetPivotEncoder(), allow).ignoringDisable(true));

        // AlLow pivot manual control
        allow.setDefaultCommand(Commands.run(() -> {
            double speed = -operator_controller.getRightX(); // Use right stick X for pivot control
            allow.manualPivotControl(speed);
        }, allow));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void disabledInit() {
        // Stop all subsystems when disabled
        elevator.stopElevator();
        allow.stopPivot();
        allow.stopRoller();
    }
}
