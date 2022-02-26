// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Turret;

public class AutoTrackingTurret extends CommandBase {

  Turret m_turret;
  double m_estimatedTargetRotation;
  
  

  /** Creates a new AutoTrackingTurret. */
  public AutoTrackingTurret(Turret turret, double estimatedTargetRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    m_turret = turret;
    m_estimatedTargetRotation = estimatedTargetRotation;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intitialize Auto Tracking Turret...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //forgot what values we wanted when writing this part
    if (m_estimatedTargetRotation > 200){
      System.out.println(m_estimatedTargetRotation);
      m_turret.setTurretDegrees(204);
    } else {
      System.out.println(m_estimatedTargetRotation);
      m_turret.setTurretDegrees(196);
    }
    

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
