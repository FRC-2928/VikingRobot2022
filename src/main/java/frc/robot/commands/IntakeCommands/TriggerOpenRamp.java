// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TriggerOpenRamp extends InstantCommand {

  private Intake m_intake;

  public TriggerOpenRamp(Intake intake) {
    addRequirements(intake);

    m_intake = intake;
  }

 
  @Override
  public void initialize() {

    m_intake.setRampState(false);
    m_intake.openRamp();

  }
}
