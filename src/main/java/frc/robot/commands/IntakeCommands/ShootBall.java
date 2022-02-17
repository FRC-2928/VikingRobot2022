// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

public class ShootBall extends CommandBase {
  
  Intake m_intake;

  /** Creates a new CheckAndShootBall. */
  public ShootBall(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.readyToShoot()) {
      // Override all brakes
      System.out.println("Ready to shoot...");
      m_intake.setFeederBrakeDisabled();
      //m_intake.setIntakeBrakeDisabled();

      // Start feeder motor at high power
      m_intake.startFeederMotor(IntakeConstants.kFeederHighSpeed);
      
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // to check: there is a ball (of right color), ramp is fully closed
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setFeederBrakeEnabled();
    m_intake.startFeederMotor(IntakeConstants.kFeederSpeed);
    m_intake.startIntakeMotor(IntakeConstants.kIntakeSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_intake.intakeCleared() && m_intake.feederCleared();
    return m_intake.feederCleared();
  }
}
