// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.Flywheel;

public class IncrementFlywheel extends CommandBase {

  double m_powerOutput;
  static final double increment = .05;
  Flywheel m_flywheel;

  /** Creates a new IncrementFlywheel. */
  public IncrementFlywheel(Flywheel flywheel) {
    m_powerOutput = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    m_flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_flywheel.getPower() < FlywheelConstants.kMotorLimit){
      m_powerOutput += increment;
      m_flywheel.setPower(m_powerOutput);
    } else {
      m_flywheel.setPower(.9);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
