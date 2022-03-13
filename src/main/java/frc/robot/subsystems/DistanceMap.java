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
        m_ticksPer100ms.put(0, 10000);
        m_ticksPer100ms.put(1, 10000);
        m_ticksPer100ms.put(2, 10000);
        m_ticksPer100ms.put(3, 10000);
        m_ticksPer100ms.put(4, 10000);
        m_ticksPer100ms.put(5, 10000);
        m_ticksPer100ms.put(6, 10000);
        m_ticksPer100ms.put(7, 10000);
        m_ticksPer100ms.put(8, 10000);
        m_ticksPer100ms.put(9, 10000);
        m_ticksPer100ms.put(10, 10000); 
        m_ticksPer100ms.put(11, 10000);
        m_ticksPer100ms.put(12, 10000);
        m_ticksPer100ms.put(13, 10000);
    }

    public int getFlywheelTicksPer100ms(int distance) {
      
       int closestTicksPer100ms = 0;

        for(var entry : m_ticksPer100ms.entrySet()){
            //not printing
            System.out.println("INSIDE FOR LOOP");
            double entryDistance = entry.getKey();
            if(entryDistance == distance){
                closestTicksPer100ms = entry.getValue();
            }
        }
        return closestTicksPer100ms;
        
    }
}    