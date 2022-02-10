// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class RegisterTurret extends CommandBase {

  Turret m_turret;
  /** Creates a new RegisterTurret. */
  public RegisterTurret(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setPower(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setSensorTicks(TurretConstants.kTurretMaxTicks);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.isLimitSwitchClosed();
  }
}
