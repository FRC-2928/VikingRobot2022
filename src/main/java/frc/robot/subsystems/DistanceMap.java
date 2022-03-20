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
        m_ticksPer100ms.put(21, 17000);
        m_ticksPer100ms.put(22, 17000);
        m_ticksPer100ms.put(23, 17000);//
        m_ticksPer100ms.put(20, 17000); //verified <20 : OUT OF RANGE
        m_ticksPer100ms.put(21, 17000);
        m_ticksPer100ms.put(22, 16400);
        m_ticksPer100ms.put(23, 15800);
        m_ticksPer100ms.put(24, 15200); 
        m_ticksPer100ms.put(25, 14600); //verified
        m_ticksPer100ms.put(26, 14000);
        m_ticksPer100ms.put(27, 13200);
        m_ticksPer100ms.put(28, 12400);
        m_ticksPer100ms.put(29, 11600);
        m_ticksPer100ms.put(30, 10800); //verified
        m_ticksPer100ms.put(31, 10000);
        m_ticksPer100ms.put(32, 9600);//verified
        m_ticksPer100ms.put(33, 9200); 
        m_ticksPer100ms.put(34, 8600);
        m_ticksPer100ms.put(35, 8200); //verified
        m_ticksPer100ms.put(36, 8000);
        m_ticksPer100ms.put(37, 7800);
        m_ticksPer100ms.put(38, 7600);
        m_ticksPer100ms.put(39, 7400);
        m_ticksPer100ms.put(40, 7200); //verified 40< : OUT OF RANGE
        m_ticksPer100ms.put(41, 7000);
        m_ticksPer100ms.put(42, 7000);
        m_ticksPer100ms.put(43, 7000);
        m_ticksPer100ms.put(44, 7000);
        m_ticksPer100ms.put(45, 7000);
        m_ticksPer100ms.put(46, 7000);
        m_ticksPer100ms.put(47, 7000);
        m_ticksPer100ms.put(48, 7000);
        m_ticksPer100ms.put(49, 7000);
        m_ticksPer100ms.put(50, 7000);
        m_ticksPer100ms.put(0, 4650);

    }

    public int getFlywheelTicksPer100ms(int distance) {
        return m_ticksPer100ms.get(distance);
    }
}    