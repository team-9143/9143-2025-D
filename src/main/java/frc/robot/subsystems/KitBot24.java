// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.KitBot24Constants;

// Controls shooter wheels.
public class KitBot24 extends SubsystemBase {
    public static final SparkMax feeder_motor = 
        new SparkMax(KitBot24Constants.kFeederID, MotorType.kBrushless);
    public static final SparkMax launcher_motor = 
        new SparkMax(KitBot24Constants.kLauncherID, MotorType.kBrushless);
    private static final SparkMaxConfig feederConfig = new SparkMaxConfig();
    private static final SparkMaxConfig launcherConfig = new SparkMaxConfig();

    static {
        // Feeder motor setup
        feederConfig.idleMode(IdleMode.kCoast)
                   .smartCurrentLimit(KitBot24Constants.kFeedCurrentLimit);
        
        feeder_motor.configure(feederConfig, 
                             ResetMode.kResetSafeParameters, 
                             PersistMode.kNoPersistParameters);

        // Launcher motor setup
        launcherConfig.idleMode(IdleMode.kBrake)
                     .smartCurrentLimit(KitBot24Constants.kLauncherCurrentLimit);
        
        launcher_motor.configure(launcherConfig, 
                               ResetMode.kResetSafeParameters, 
                               PersistMode.kNoPersistParameters);
    }

    private static final KitBot24 m_kitbot = new KitBot24();

    // Returns the singleton instance
    public static KitBot24 getInstance() {
        return m_kitbot;
    }

    // Returns a command to intake a game piece using both wheels.
    public Command getIntakeCommand() {
        return Commands.startEnd(
            () -> {
                feeder_motor.set(KitBot24Constants.kIntakeFeederSpeed);
                launcher_motor.set(KitBot24Constants.kIntakeLauncherSpeed);
            },
            () -> {
                feeder_motor.set(0);
                launcher_motor.set(0);
            },
            this);
    }

    // Returns a command to shoot a game piece using both wheels.
    public Command getShootCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> launcher_motor.set(KitBot24Constants.kShootLauncherSpeed)),
            Commands.waitSeconds(KitBot24Constants.kFeederDelay),
            Commands.startEnd(
                () -> {
                    launcher_motor.set(KitBot24Constants.kShootLauncherSpeed);
                    feeder_motor.set(KitBot24Constants.kShootFeederSpeed);
                },
                () -> {
                    launcher_motor.set(0);
                    feeder_motor.set(0);
                },
                this));
    }

    public void stop() {
        feeder_motor.set(0);
        launcher_motor.set(0);
    }
}