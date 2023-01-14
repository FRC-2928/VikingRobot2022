// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AutoTrackingTurret extends CommandBase {

  Turret m_turret;
  Rotation2d m_estimatedTargetRotation;

  /** Creates a new AutoTrackingTurret. */
  public AutoTrackingTurret(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    m_turret = turret;
    m_estimatedTargetRotation = m_turret.getEstimatedTargetRotation();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intitialize Auto Tracking Turret...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetAngle = m_turret.getTargetHorizontalOffset();
    double turretAngle = m_turret.getTurretDegrees();
    double newAngle = turretAngle + targetAngle;

    if (m_turret.angleInRange(newAngle)) {
      m_turret.setTurretDegrees(-newAngle);
    } 
    else 
    {}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending Auto Tracking Turret...");
    System.out.println(m_estimatedTargetRotation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
