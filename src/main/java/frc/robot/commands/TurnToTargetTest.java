// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TurnToTargetTest extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Turret m_turret;
  private final static double kP = 0.05;

  /** Creates a new TurnToTargetTest. */
  public TurnToTargetTest(Drivetrain drivetrain, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, turret);
    m_drivetrain = drivetrain;
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measurement = m_turret.targetHorizontalOffset();
    SmartDashboard.putNumber("Measurement", measurement);
    double output = measurement * kP;
    m_drivetrain.drive(0, -output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
