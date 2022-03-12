
package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.DistanceMap;
import frc.robot.subsystems.Limelight;

/*
 * Spins up the flywheel to desired RPM
 */

public class SpinUpFlywheel extends CommandBase {

    // The subsystem the command runs on
    private final Flywheel m_flywheel;
    private double m_flywheelRPM;

    public SpinUpFlywheel(Flywheel flywheel, Limelight limelight) {
        addRequirements(flywheel);
        m_flywheel = flywheel;
        double distance = limelight.getTargetDistance();
        m_flywheelRPM = calculateFlywheelRPM(distance);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_flywheel.setPosition(m_flywheelRPM);
    }

    // Calculate the RPM based on the distance
    private static double calculateFlywheelRPM(double distance) {
        double rpm = DistanceMap.getInstance().getFlywheelRPM(distance);
        return rpm;
    }

}