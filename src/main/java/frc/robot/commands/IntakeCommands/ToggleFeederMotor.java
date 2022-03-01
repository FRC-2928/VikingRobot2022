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
public class ToggleFeederMotor extends InstantCommand {

  Intake m_intake;
  
  /**
   * switches the feeder motor between off and 20% power
   * @param intake
   */
  public ToggleFeederMotor(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_intake.isFeederMotorOn()){
      System.out.println("Stop Feeder motor");
      m_intake.stopFeederMotor();
    } else {
      System.out.println("Start Feeder motor");
      m_intake.startFeederMotor(IntakeConstants.kFeederSpeed);
    }
  }
}
