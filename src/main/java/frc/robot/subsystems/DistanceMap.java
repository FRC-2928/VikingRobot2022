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
        m_tickspersecond.put(0, 8500);
        m_tickspersecond.put(1, 8500);
        m_tickspersecond.put(2, 8500);
        m_tickspersecond.put(3, 8500);
        m_tickspersecond.put(4, 8500);
        m_tickspersecond.put(5, 10000);
        m_tickspersecond.put(6, 10000);
        m_tickspersecond.put(7, 12000);
        m_tickspersecond.put(8, 12000);
        m_tickspersecond.put(9, 12000);
        m_tickspersecond.put(10, 15000); 
        m_tickspersecond.put(11, 15000);
        m_tickspersecond.put(12, 15000);
        m_tickspersecond.put(13, 15000);
    }

    public int getFlywheelTicksPerSecond(int distance) {
        return m_tickspersecond.get(distance);
    }
}    