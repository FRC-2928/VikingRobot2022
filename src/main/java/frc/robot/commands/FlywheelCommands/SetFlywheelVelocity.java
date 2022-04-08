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
  
    //addRequirements(flywheel);
    m_flywheel = flywheel;
    m_turret = turret;
    
  }

  public void initialize() {
    super.initialize();  

    int distance = m_turret.getTargetVerticalOffset();
    int flywheelTicksPer100ms = m_flywheel.calculateFlywheelTicksPer100ms(distance);
    // System.out.println("distance: " + distance);
    // System.out.println("ticks: " + flywheelTicksPer100ms);

    m_flywheel.setPidGains(flywheelTicksPer100ms);
    m_flywheel.setAdjustableVelocity(flywheelTicksPer100ms);
  }

  public void end(boolean interrupted) {}
}

