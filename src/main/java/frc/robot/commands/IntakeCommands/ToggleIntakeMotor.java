// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleIntakeMotor extends InstantCommand {
  
  Intake m_intake;

  /**
   * switches the intake motor between on and off
   * @param intake
   */
  public ToggleIntakeMotor(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if(m_intake.isIntakeMotorOn()){
      // We're doing a hard stop here.
      m_intake.setIntakeMotorStop(true);
      m_intake.stopIntakeMotor();
      System.out.println("Intake motor stopped...");
    } else {
      m_intake.setIntakeMotorStop(false);
      m_intake.startIntakeMotor(IntakeConstants.kIntakeSpeed);
      System.out.println("Intake motor started...");
    }
  }
}
