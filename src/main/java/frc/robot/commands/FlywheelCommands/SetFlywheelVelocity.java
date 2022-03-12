package frc.robot.commands.FlywheelCommands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFlywheelVelocity extends InstantCommand {
  Flywheel m_flywheel;
  Turret m_turret;

  /** Creates a new TurnToTarget. */
  public SetFlywheelVelocity(Flywheel flywheel, Turret turret) {
  
    addRequirements(flywheel);
    m_flywheel = flywheel;
    m_turret = turret;
    
    // Configure additional PID options by calling `getController` here.
  }

  public void initialize() {
    super.initialize();  
    System.out.println("IN PID");  
    
    double distance = m_turret.getRoundedVerticalOffset();
    double flywheelTicksPerSecond = m_flywheel.calculateFlywheelTicksPerSecond(distance);
    m_flywheel.setVelocity(flywheelTicksPerSecond);
  }

  public void end(boolean interrupted) {}
}

