// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

public class ShootBall extends CommandBase {
  
  Intake m_intake;
  Flywheel m_flywheel;
  Timer m_shootTimer = new Timer();

  /** Creates a new ShootBall. */
  public ShootBall(Intake intake, Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_intake = intake;
    m_flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.isRampClosed()) {
      // Override all brakes
      System.out.println("Ready to shoot...");
      
     //m_intake.setIntakeBrakeDisabled();

      // boolean flywheelSpun = false;
      // while(flywheelSpun == false){
      //   if(m_flywheel.isFlyWheelUpToSpeed()){
      //   // Start feeder motor at high power
      //     flywheelSpun = true;
      //     m_intake.setFeederBrakeDisabled();
      //     m_intake.startFeederMotor(IntakeConstants.kFeederHighSpeed);
      //     System.out.println("SHOT BALL");
      //   }else{
      //     new WaitCommand(.25);
      //     System.out.println("Still Spinning");
      //   }
      // }
    } 

    m_shootTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_flywheel.isFlyWheelUpToSpeed()){
      // Start feeder motor at high power
      m_intake.setFeederBrakeDisabled();
      m_intake.startFeederMotor(IntakeConstants.kFeederHighSpeed);    
      m_shootTimer.start();
    }
  }                                                           

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_intake.setFeederBrakeEnabled();
    m_intake.setIntakeBrakeDisabled();
    m_intake.startFeederMotor(IntakeConstants.kFeederSpeed);
    m_intake.startIntakeMotor(IntakeConstants.kIntakeSpeed);
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_intake.intakeCleared() && m_intake.feederCleared();
    return (!(m_intake.isFeederSwitchActivated()) || m_shootTimer.hasElapsed(1));
    
  }
}
