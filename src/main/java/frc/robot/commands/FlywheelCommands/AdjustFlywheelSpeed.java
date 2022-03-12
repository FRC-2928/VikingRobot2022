package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;

public class AdjustFlywheelSpeed extends CommandBase {

  Flywheel m_flywheel;
  Turret m_turret;

  Double m_estimatedTargetSpeed;


  /** Creates a new AutoTrackingTurret. */
  public AdjustFlywheelSpeed(Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    m_flywheel = flywheel;
    m_estimatedTargetSpeed = m_flywheel.getRPM();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intitialize Adjusting Flywheel Speeds...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetSpeed = m_turret.getTargetVerticalOffset();
    double flywheelSpeed = m_flywheel.getVelocity();
    double newSpeed = flywheelSpeed + targetSpeed;

    if (m_flywheel.speedInRange(newSpeed)) {
      m_flywheel.setVelocity(-newSpeed);
    } 
    else 
    {}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending Adjusting Flywheel Speeds...");
    System.out.println(m_estimatedTargetSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
