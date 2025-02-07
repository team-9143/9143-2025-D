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
import frc.robot.Constants.Amper24Constants;

public class Amper24 extends SubsystemBase {

    public static final SparkMax amper_motor = 
        new SparkMax(Amper24Constants.kAmperID, MotorType.kBrushed);
    private static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    static {
        // Amper motor setup
        motorConfig.idleMode(IdleMode.kBrake)
                  .smartCurrentLimit(Amper24Constants.kAmperCurrentLimit);
        
        amper_motor.configure(motorConfig, 
                            ResetMode.kResetSafeParameters, 
                            PersistMode.kNoPersistParameters);
    }

    private static final Amper24 m_amper = new Amper24();

    // Returns the singleton instance
    public static Amper24 getInstance() {
        return m_amper;
    }

    public boolean isHoldPositionActive = false;

    // Method to set the motor for intake mode.
    public Command getIntakeCommand() {
        return Commands.startEnd(
            () -> amper_motor.setVoltage(Amper24Constants.kAmperIntakeSpeed * 12),
            () -> amper_motor.set(0),
            this);
    }

    // Method to set the motor for scoring mode.
    public Command getScoreCommand() {
        return Commands.startEnd(
            () -> amper_motor.setVoltage(Amper24Constants.kAmperScoreSpeed * 12),
            () -> amper_motor.set(0),
            this);
    }

    // Method to set the motor at stall speed.
    public Command getHoldPositionCommand() {
        return Commands.startEnd(
            () -> amper_motor.setVoltage(Amper24Constants.kAmperHoldPositionSpeed * 12),
            () -> amper_motor.set(0),
            this);
    }

    // Method to stop the motor
    public void stop() {
        amper_motor.set(0);
    }

    public void log() {}
}