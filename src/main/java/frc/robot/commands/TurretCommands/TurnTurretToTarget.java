// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnTurretToTarget extends PIDCommand {
  Turret m_turret;

  /** Creates a new TurnToTarget. */
  public TurnTurretToTarget(Turret turret) {
    super(
        // The controller that the command will use
        new PIDController(0.018, 0, 0.001),
        // This should return the measurement
        () -> turret.getTargetHorizontalOffset(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          turret.setPower(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    m_turret = turret;
    
    // Configure additional PID options by calling `getController` here.
  }

  public void initialize() {
    super.initialize();  
    System.out.println("IN PID");  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    //SmartDashboard.putNumber("end heading", m_drivetrain.getHeading());
    System.out.println("Done PID");
  }

}
