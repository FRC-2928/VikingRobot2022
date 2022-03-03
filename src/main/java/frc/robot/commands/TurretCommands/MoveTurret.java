// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {

  Turret m_turret;
  double m_direction;
  double m_currentAngle = 0;

  /** Creates a new MoveTurret. */
  /**
   * moves the turret at half power in the desired direction
   * left direction is clockwise, right is counter
   * @param turret
   * @param direction negative value for left, positive for right
   */
  public MoveTurret(Turret turret, double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    m_turret = turret;
    m_direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentAngle = m_turret.getTurretDegrees();
    System.out.println("Initialize Move turret " + m_currentAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Compute and set the new angle
    m_currentAngle += m_direction;
    // m_turret.setTurretDegrees(m_currentAngle);

    if(m_direction > 0){  
      m_turret.setPower(.3);
    } else {
      m_turret.setPower(-.3);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
