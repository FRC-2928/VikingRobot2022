package frc.robot.commands.FlywheelCommands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DistanceMap;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * resets the flywheel velocity based on distance from the target using the limelight and flywheel, routed through distance map
 */
public class SetFlywheelAuto extends InstantCommand {
  Flywheel m_flywheel;
  Turret m_turret;

  
  public SetFlywheelAuto(Flywheel flywheel, Turret turret) {
  
    //addRequirements(flywheel);
    m_flywheel = flywheel;
    m_turret = turret;
    
  }

  public void initialize() {
    super.initialize();  


    m_flywheel.setAdjustableVelocity(6100);
  }

  public void end(boolean interrupted) {}


 
  
}

