// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class IncrementFlywheel extends CommandBase {

  static final double increment = .005;
  Flywheel m_flywheel;

  /** Creates a new IncrementFlywheel. */
  public IncrementFlywheel(Flywheel flywheel) {
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

    m_flywheel.incrementVelocity(increment);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Increment flywheel ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
