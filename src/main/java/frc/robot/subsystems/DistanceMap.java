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
        m_ticksPer100ms.put(13, 20000);
        m_ticksPer100ms.put(14, 20000);
        m_ticksPer100ms.put(15, 18000);
        m_ticksPer100ms.put(16, 14000);
        m_ticksPer100ms.put(17, 12000);
        m_ticksPer100ms.put(18, 11000);
        m_ticksPer100ms.put(19, 10000); //verified
        m_ticksPer100ms.put(20, 9800); //verified <20 : OUT OF RANGE
        m_ticksPer100ms.put(21, 9600);
        m_ticksPer100ms.put(22, 9400);
        m_ticksPer100ms.put(23, 9200);
        m_ticksPer100ms.put(24, 9000); 
        m_ticksPer100ms.put(25, 8900); //verified
        m_ticksPer100ms.put(26, 8800);
        m_ticksPer100ms.put(27, 8600);
        m_ticksPer100ms.put(28, 8400);
        m_ticksPer100ms.put(29, 8200);
        m_ticksPer100ms.put(30, 8100); //verified
        m_ticksPer100ms.put(31, 8000);
        m_ticksPer100ms.put(32, 8000);//verified
        m_ticksPer100ms.put(33, 8000); 
        m_ticksPer100ms.put(34, 8000);
        m_ticksPer100ms.put(35, 8000); //verified
        m_ticksPer100ms.put(36, 8000);
        m_ticksPer100ms.put(37, 8000);
        m_ticksPer100ms.put(38, 8000);
        m_ticksPer100ms.put(39, 8000);
        m_ticksPer100ms.put(40, 8000); //verified 40< : OUT OF RANGE
        m_ticksPer100ms.put(41, 8000);
        m_ticksPer100ms.put(42, 8000);
        m_ticksPer100ms.put(43, 8000);
        m_ticksPer100ms.put(44, 8000);
        m_ticksPer100ms.put(45, 8000);
        m_ticksPer100ms.put(46, 8000);
        m_ticksPer100ms.put(47, 8000);
        m_ticksPer100ms.put(48, 8000);
        m_ticksPer100ms.put(49, 8000);
        m_ticksPer100ms.put(50, 8000);
        m_ticksPer100ms.put(0, 4650);

    }

    public int getFlywheelTicksPer100ms(int distance) {
        return m_ticksPer100ms.get(distance);
    }
}    