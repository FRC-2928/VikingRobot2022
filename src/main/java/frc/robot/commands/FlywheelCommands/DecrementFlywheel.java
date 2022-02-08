// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class DecrementFlywheel extends CommandBase {

  Flywheel m_flywheel;
  static final double decrement = .05;
  double m_powerOutput;
  
  /** Creates a new DecrementFlywheel. */
  public DecrementFlywheel(Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    m_flywheel = flywheel;
    m_powerOutput = m_flywheel.getPower();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_powerOutput <= 0){
      m_flywheel.setPower(0);
    } else{
      m_flywheel.setPower(m_powerOutput);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
