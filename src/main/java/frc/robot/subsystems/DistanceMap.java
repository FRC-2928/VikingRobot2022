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
        m_tickspersecond.put(0, 15000);
        m_tickspersecond.put(1, 15000);
        m_tickspersecond.put(2, 15000);
        m_tickspersecond.put(3, 15000);
        m_tickspersecond.put(4, 15000);
        m_tickspersecond.put(5, 12000);
        m_tickspersecond.put(6, 12000);
        m_tickspersecond.put(7, 10000);
        m_tickspersecond.put(8, 10000);
        m_tickspersecond.put(9, 10000);
        m_tickspersecond.put(10, 8500); 
        m_tickspersecond.put(11, 8500);
        m_tickspersecond.put(12, 8500);
        m_tickspersecond.put(13, 8500);
    }

    public int getFlywheelTicksPerSecond(int distance) {
        return m_tickspersecond.get(distance);
    }
}    