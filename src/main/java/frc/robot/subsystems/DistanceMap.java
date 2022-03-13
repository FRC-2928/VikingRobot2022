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

    private final Map<Integer, Integer> m_ticksPer100ms = new HashMap<>();

    public void loadMaps() {
        m_ticksPer100ms.put(0, 15000);
        m_ticksPer100ms.put(1, 15000);
        m_ticksPer100ms.put(2, 15000);
        m_ticksPer100ms.put(3, 15000);
        m_ticksPer100ms.put(4, 15000);
        m_ticksPer100ms.put(5, 12000);
        m_ticksPer100ms.put(6, 12000);
        m_ticksPer100ms.put(7, 10000);
        m_ticksPer100ms.put(8, 10000);
        m_ticksPer100ms.put(9, 10000);
        m_ticksPer100ms.put(10, 8500); 
        m_ticksPer100ms.put(11, 8500);
        m_ticksPer100ms.put(12, 8500);
        m_ticksPer100ms.put(13, 8500);
    }

    public int getFlywheelTicksPer100ms(int distance) {
        return m_ticksPer100ms.get(distance);
    }
}    