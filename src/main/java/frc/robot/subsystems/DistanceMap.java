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

    private final Map<Integer, Integer> m_tickspersecond = new HashMap<>();

    public void loadMaps() {
        m_tickspersecond.put(0, 0);
        m_tickspersecond.put(1, 10000);
        m_tickspersecond.put(2, 10000);
        m_tickspersecond.put(3, 10000);
        m_tickspersecond.put(4, 10000);
        m_tickspersecond.put(5, 10000);
        m_tickspersecond.put(6, 10000);
        m_tickspersecond.put(7, 10000);
        m_tickspersecond.put(8, 10000);
        m_tickspersecond.put(9, 10000);
        m_tickspersecond.put(10, 10000); 
        m_tickspersecond.put(11, 10000);
        m_tickspersecond.put(12, 10000);
        m_tickspersecond.put(13, 10000);
    }

    public double getFlywheelTicksPerSecond(double distance) {
        double closestOffset = Double.POSITIVE_INFINITY;
        double closestTicksPerSecond = 0;

        for(var entry : m_tickspersecond.entrySet()){
            double entryDistance = entry.getKey();
            double entryOffset = Math.abs(entryDistance - distance);
            if(entryOffset < closestOffset){
                closestOffset = entryOffset;
                closestTicksPerSecond = entry.getValue();
            }
        }

        SmartDashboard.putNumber("Distance Map Distance(flywheel)", distance);
        SmartDashboard.putNumber("Distance Map TicksPerSecond", closestTicksPerSecond);
        return closestTicksPerSecond;
    }
}    