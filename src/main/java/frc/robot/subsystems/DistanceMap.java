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
        m_tickspersecond.put(1, 1000);
        m_tickspersecond.put(2, 2000);
        m_tickspersecond.put(3, 3000);
        m_tickspersecond.put(4, 4000);
        m_tickspersecond.put(5, 5000);
        m_tickspersecond.put(6, 6000);
        m_tickspersecond.put(7, 7000);
        m_tickspersecond.put(8, 8000);
        m_tickspersecond.put(9, 9000);
        m_tickspersecond.put(10, 10000); 
        m_tickspersecond.put(11, 11000);
        m_tickspersecond.put(12, 12000);
        m_tickspersecond.put(13, 13000);
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