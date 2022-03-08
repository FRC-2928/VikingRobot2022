// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class MoveToShootingDistance extends CommandBase {
  Turret m_turret;
  Drivetrain m_drivetrain;
  double m_offset;
  /** Creates a new MoveToShootingDistance. */
  public MoveToShootingDistance(Turret turret, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(drivetrain);
    m_turret = turret;
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_offset = m_turret.getTargetVerticalOffset();
    if (m_offset < 5){
      m_drivetrain.drive(.15, 0);
    } else if (m_offset > 5.5) {
      m_drivetrain.drive(-.15, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    //SmartDashboard.putNumber("end heading", m_drivetrain.getHeading());
    m_drivetrain.stopDrivetrain();
    m_drivetrain.enableMotorSafety();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_offset > 5 && m_offset < 5.5);
  }
}
