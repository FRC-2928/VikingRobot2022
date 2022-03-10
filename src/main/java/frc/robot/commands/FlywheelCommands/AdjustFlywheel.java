// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DistanceMap;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Limelight;

/*
 * Spins up the flywheel to desired RPM
 */

public class AdjustFlywheel extends CommandBase {

    
    Flywheel m_flywheel;
    private double m_flywheelRPM;

    public AdjustFlywheel(Flywheel flywheel, Limelight limelight) {
        addRequirements(flywheel);
        m_flywheel = flywheel;
        double distance = limelight.getTargetDistance();
        m_flywheelRPM = calculateFlywheelRPM(distance);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_flywheel.setPosition(m_flywheelRPM);
    }

    // Calculate the RPM based on the distance
    private static double calculateFlywheelRPM(double distance) {
        double rpm = DistanceMap.getInstance().getFlywheelRPM(distance);
        return rpm;
    }

}
