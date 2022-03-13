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
        
        m_ticksPer100ms.put(10, 20000);
        m_ticksPer100ms.put(11, 20000);
        m_ticksPer100ms.put(12, 20000);
        m_ticksPer100ms.put(13, 20000);
        m_ticksPer100ms.put(14, 20000);
        m_ticksPer100ms.put(15, 20000); 
        m_ticksPer100ms.put(16, 20000);
        m_ticksPer100ms.put(17, 20000);
        m_ticksPer100ms.put(18, 20000);
        m_ticksPer100ms.put(19, 20000);
        m_ticksPer100ms.put(20, 17000); //verified
        m_ticksPer100ms.put(21, 16400);
        m_ticksPer100ms.put(22, 15800);
        m_ticksPer100ms.put(23, 15200);
        m_ticksPer100ms.put(24, 14600); 
        m_ticksPer100ms.put(25, 14000); //verified
        m_ticksPer100ms.put(26, 13200);
        m_ticksPer100ms.put(27, 12400);
        m_ticksPer100ms.put(28, 11600);
        m_ticksPer100ms.put(29, 10800);
        m_ticksPer100ms.put(30, 10000); //verified
        m_ticksPer100ms.put(31, 9600);
        m_ticksPer100ms.put(32, 9200);
        m_ticksPer100ms.put(33, 8800); 
        m_ticksPer100ms.put(34, 8400);
        m_ticksPer100ms.put(35, 8000); //verified
        m_ticksPer100ms.put(36, 7800);
        m_ticksPer100ms.put(37, 7600);
        m_ticksPer100ms.put(38, 7400);
        m_ticksPer100ms.put(39, 7200);
        m_ticksPer100ms.put(40, 6800); //verified
        m_ticksPer100ms.put(41, 6600);
        m_ticksPer100ms.put(42, 6400);
        m_ticksPer100ms.put(43, 6200);
        m_ticksPer100ms.put(44, 6000);
        m_ticksPer100ms.put(45, 6000);
        m_ticksPer100ms.put(46, 6000);
        m_ticksPer100ms.put(47, 6000);
        m_ticksPer100ms.put(48, 6000);
        m_ticksPer100ms.put(49, 6000);
        m_ticksPer100ms.put(50, 6000);
        m_ticksPer100ms.put(0, 4500);

    }

    public int getFlywheelTicksPer100ms(int distance) {
        return m_ticksPer100ms.get(distance);
    }
}    