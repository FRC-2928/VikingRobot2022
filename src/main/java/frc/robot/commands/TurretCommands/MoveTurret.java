// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {

  Turret m_turret;
  double m_direction;

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
    System.out.println("Intitialize Moving turret...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_direction > 0){
      System.out.println("Moving turret left...");
      m_turret.setPower(.2);
    } else {
      System.out.println("Moving turret right...");
      m_turret.setPower(-.2);
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
