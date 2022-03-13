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

  
  public SetFlywheelVelocity(Flywheel flywheel, Turret turret) {
  
    addRequirements(flywheel);
    m_flywheel = flywheel;
    m_turret = turret;
    
  }

  public void initialize() {
    super.initialize();  

    int distance = m_turret.getTargetVerticalOffset();
    int flywheelTicksPerSecond = m_flywheel.calculateFlywheelTicksPerSecond(distance);
    System.out.println("distance: " + distance);
    System.out.println("ticks: " + flywheelTicksPerSecond);

    m_flywheel.setVelocity(flywheelTicksPerSecond);
  }

  public void end(boolean interrupted) {}
}

