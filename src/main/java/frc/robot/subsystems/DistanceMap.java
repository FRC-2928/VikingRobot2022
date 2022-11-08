package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

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
        m_ticksPer100ms.put(13, 18000);
        m_ticksPer100ms.put(14, 18000);
        m_ticksPer100ms.put(15, 18000);
        m_ticksPer100ms.put(16, 14000);
        m_ticksPer100ms.put(17, 12000);
        m_ticksPer100ms.put(18, 11000);
        m_ticksPer100ms.put(19, 10000); 
        m_ticksPer100ms.put(20, 9800); 
        m_ticksPer100ms.put(21, 9000);
        m_ticksPer100ms.put(22, 8800);
        m_ticksPer100ms.put(23, 8600);
        m_ticksPer100ms.put(24, 8400); 
        m_ticksPer100ms.put(25, 8200); 
        m_ticksPer100ms.put(26, 8100);
        m_ticksPer100ms.put(27, 8100);
        m_ticksPer100ms.put(28, 7800);
        m_ticksPer100ms.put(29, 7400);
        m_ticksPer100ms.put(30, 7400);
        m_ticksPer100ms.put(31, 6800);
        m_ticksPer100ms.put(32, 6400);
        m_ticksPer100ms.put(33, 6300); 
        m_ticksPer100ms.put(34, 6100);
        m_ticksPer100ms.put(35, 6100); 
        m_ticksPer100ms.put(36, 6100);
        m_ticksPer100ms.put(37, 6000);
        m_ticksPer100ms.put(38, 6000);
        m_ticksPer100ms.put(39, 6000);
        m_ticksPer100ms.put(40, 5800); 
        m_ticksPer100ms.put(41, 5800);
        m_ticksPer100ms.put(42, 5800);
        m_ticksPer100ms.put(43, 5600);
        m_ticksPer100ms.put(44, 5600);
        m_ticksPer100ms.put(45, 5500);
        m_ticksPer100ms.put(46, 5500);
        m_ticksPer100ms.put(47, 5500);
        m_ticksPer100ms.put(48, 5500);
        m_ticksPer100ms.put(49, 5400);
        m_ticksPer100ms.put(50, 5400);
        //5000 for low goal with 60% ratio good
        m_ticksPer100ms.put(0, 3000);

    }

    public int getFlywheelTicksPer100ms(int distance) {
        return m_ticksPer100ms.get(distance);
    }
}    