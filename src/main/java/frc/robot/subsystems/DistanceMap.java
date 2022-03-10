package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for storing LimelightData
 */
public class DistanceMap {

    private static DistanceMap m_instance;

    public static DistanceMap getInstance() {
        if (m_instance == null) {
            m_instance = new DistanceMap();
        }
        return m_instance;
    }

    private final Map<Double, Integer> m_degrees = new HashMap<>();
    private final Map<Double, Integer> m_rpm = new HashMap<>();

    public void loadMaps() {
        // Load degrees
        m_degrees.put(1.0, 1);
       

        // Load RPM
        m_rpm.put(1.0, 1000);
        
    }

    public double getHoodDegrees(double distance) {
        double closestOffset = Double.POSITIVE_INFINITY;
        double closestDegrees = 0;

        for(var entry : m_degrees.entrySet()){
            double entryDistance = entry.getKey();
            double entryOffset = Math.abs(entryDistance - distance);
            if(entryOffset < closestOffset){
                closestOffset = entryOffset;
                closestDegrees = entry.getValue();
            }
        }

        SmartDashboard.putNumber("Distance Map Distance(hood)", distance);
        SmartDashboard.putNumber("Distance Map Degrees", closestDegrees);
        return closestDegrees;
    }

    public double getFlywheelRPM(double distance) {
        double closestOffset = Double.POSITIVE_INFINITY;
        double closestRPM = 0;

        for(var entry : m_rpm.entrySet()){
            double entryDistance = entry.getKey();
            double entryOffset = Math.abs(entryDistance - distance);
            if(entryOffset < closestOffset){
                closestOffset = entryOffset;
                closestRPM = entry.getValue();
            }
        }

        SmartDashboard.putNumber("Distance Map Distance(flywheel)", distance);
        SmartDashboard.putNumber("Distance Map RPM", closestRPM);
        return closestRPM;
    }
}    