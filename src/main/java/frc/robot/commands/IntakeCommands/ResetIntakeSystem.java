// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

import frc.robot.Constants.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetIntakeSystem extends InstantCommand {

  Intake m_intake;
  public ResetIntakeSystem(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_intake.setFeederBrakeEnabled();
    m_intake.startFeederMotor(IntakeConstants.kFeederSpeed);
    m_intake.setIntakeBrakeDisabled();
    
    // m_intake.setIntakeBrakeDisabled();
    m_intake.startIntakeMotor(IntakeConstants.kIntakeSpeed);
  }
}
