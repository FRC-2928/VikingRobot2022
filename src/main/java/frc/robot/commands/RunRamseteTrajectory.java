package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RunRamseteTrajectory extends RamseteCommand {

  Drivetrain m_drivetrain;
  Trajectory m_trajectory;

  /**
   * Creates a new RamseteTrajectoryCommand.
   */
  public RunRamseteTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
    super(
      trajectory,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      DrivetrainConstants.kDriveKinematics,
      drivetrain::setOutputMetersPerSecond,
      drivetrain
    );
    m_drivetrain = drivetrain;
    m_trajectory = trajectory;
  }

  public void initialize() {
    super.initialize();
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
    m_drivetrain.disableMotorSafety();    
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    m_drivetrain.stopDrivetrain();
    m_drivetrain.enableMotorSafety();
  }

}
