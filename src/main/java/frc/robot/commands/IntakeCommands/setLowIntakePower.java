// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class setLowIntakePower extends CommandBase {

  Intake m_intake;
  Drivetrain m_drivetrain;
  
  /** Creates a new setLowIntakePower. */
  public setLowIntakePower(Intake intake, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_intake = intake;
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.startIntakeMotor(IntakeConstants.kIntakeLowSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.startIntakeMotor(IntakeConstants.kIntakeSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.getMotorOutputPercent() < .75;
  }

}
