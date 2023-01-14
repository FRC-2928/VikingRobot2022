// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectBall extends SequentialCommandGroup {

  Intake m_intake;
  
  /** Creates a new EjectBall. */
  public EjectBall(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new OpenRamp(intake),  
                new WaitCommand(.1),
                new TriggerEjectBall(intake),
                new WaitCommand(.4),
                new CloseRamp(intake),
                
                new DisableIntakeBrake(intake),
                new StartIntakeMotor(intake),
                new WaitCommand(.2));
                
                
  }
}
