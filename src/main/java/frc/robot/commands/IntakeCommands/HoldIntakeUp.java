// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class HoldIntakeUp extends CommandBase {

  Intake m_intake;
  
  /** Creates a new HoldIntakeUp. */
  public HoldIntakeUp(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeOut();
    m_intake.dontAllowIntakeUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // 10/10 reversed these two lines
    m_intake.allowIntakeUp();
    m_intake.setIntakeUp();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
