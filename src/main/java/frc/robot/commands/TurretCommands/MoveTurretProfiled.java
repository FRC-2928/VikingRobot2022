// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveTurretProfiled extends ProfiledPIDCommand {
  Turret m_turret;
  double m_angle;

  /** Creates a new MoveTurretProfiled. */
  public MoveTurretProfiled(Turret turret, double angle) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.5,
            0,
            0.01,
            // The motion profile constraints
            TurretConstants.kTrapezoidProfileConstraints),
        // This should return the measurement
        () -> turret.getTurretDegrees(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(angle, 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          turret.setPower(-output);
        });

    m_turret = turret;
    m_angle = angle;
    addRequirements(turret);
    // setTolerance only work for regular PID
    // getController().setTolerance(TurretConstants.kangleToleranceDegrees,0);
  }

  public void initialize(){
    super.initialize();
    // getController().reset(0,0);
    System.out.println("MoveTurretProfiled " + m_angle);
    System.out.println("Angle start " + m_turret.getTurretDegrees());
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    m_turret.setPower(0);
    // getController().reset(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
