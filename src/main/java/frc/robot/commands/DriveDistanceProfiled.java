// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceProfiled extends ProfiledPIDCommand {

  Drivetrain m_drivetrain;
  /** Creates a new RunProfilePID. */
  public DriveDistanceProfiled(double targetDistance, Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            32,
            0,
            0,
            // The motion profile constraints
            DrivetrainConstants.kTrapezoidProfileConstraints),
        // This should return the measurement
        () -> drivetrain.getAvgDistanceMeters(), 
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(targetDistance, 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.setOutputMetersPerSecond(output, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.

        m_drivetrain = drivetrain;
  }

  public void initialize(){
    super.initialize();
    m_drivetrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
