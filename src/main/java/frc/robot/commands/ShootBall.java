// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {

  Intake m_intake;
  Turret m_turret;
  Flywheel m_flywheel;

  /** Creates a new ShootBall. */
  public ShootBall(Intake intake, Turret turret, Flywheel flywheel) {
    m_intake = intake;
    m_turret = turret;
    m_flywheel = flywheel;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TriggerShootBall(m_intake, m_turret, m_flywheel), 
                new WaitCommand(.02), 
                new SetFeederCleared(m_intake));
  }
}
