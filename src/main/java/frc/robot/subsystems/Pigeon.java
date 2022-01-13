package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants.RobotMap;

/**
 * Utility class responsible for the gyro
 */
public class Pigeon{
    private PigeonIMU m_pigeon;

    public Pigeon(){
        m_pigeon = new PigeonIMU(RobotMap.kPigeonIMU);
    }

    public double getYaw(){
        double ypr[] = {0, 0, 0};
        m_pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public void resetGyro(){
        m_pigeon.setYaw(0);
    }
}