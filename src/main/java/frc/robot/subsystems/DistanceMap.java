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

    private final Map<Double, Integer> m_rpm = new HashMap<>();

    public void loadMaps() {
        
        // Load RPM
        m_rpm.put(5.0, 8000);
        m_rpm.put(6.0, 9000);
        m_rpm.put(7.0, 10000);
        m_rpm.put(8.0, 11000);
        m_rpm.put(9.0, 12000);
        m_rpm.put(10.0, 13000);
        
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