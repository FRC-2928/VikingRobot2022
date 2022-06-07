// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.Flywheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleFlywheel extends InstantCommand {

  Flywheel m_flywheel;
  
  public ToggleFlywheel(Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    m_flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_flywheel.isFlywheelMotorOn()){
      System.out.println("Flywheel was on..");
      m_flywheel.setAdjustableVelocity(0);
    } else {
      System.out.println("Flywheel was off..");
      m_flywheel.setAdjustableVelocity(5000);
    }
  }
}
