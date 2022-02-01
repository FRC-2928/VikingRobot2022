// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightData;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final Limelight m_turretLimelight = new Limelight();
  private LimelightData m_limelightData = m_turretLimelight.getLimelightData();
  private final TalonSRX m_turretMotor  = new TalonSRX(Constants.RobotMap.kTurretSparkMax);

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Turret() {
    configMotors();
    resetEncoders();
  }

  public void configMotors(){
      //Reset settings for safety
     m_turretMotor.configFactoryDefault();

      //Sets voltage compensation to 12, used for percent output
     m_turretMotor.configVoltageCompSaturation(10);
     m_turretMotor.enableVoltageCompensation(true);

      //Setting just in case
     m_turretMotor.configNominalOutputForward(0);
     m_turretMotor.configNominalOutputReverse(0);
     m_turretMotor.configPeakOutputForward(1);
     m_turretMotor.configPeakOutputReverse(-1);

     m_turretMotor.configOpenloopRamp(0.1);

      //Setting deadband(area required to start moving the motor) to 1%
     m_turretMotor.configNeutralDeadband(0.01);

      //Set to brake mode, will brake the motor when no power is sent
     m_turretMotor.setNeutralMode(NeutralMode.Coast);

      /** 
       * Setting input side current limit (amps)
       * 45 continious, 80 peak, 30 millieseconds allowed at peak
       * 40 amp breaker can support above 40 amps for a little bit
       * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
       */
     m_turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

      //Either using the integrated Falcon sensor or an external one, will change if needed
     m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
  }

  public void resetEncoders(){
    m_turretMotor.setSelectedSensorPosition(0);
  }

  public double encoderTicksToDegrees(double encoderTicks) {
    double turretRotations = encoderTicks / (TurretConstants.kEncoderCPR * TurretConstants.kGearRatio);
    return turretRotations * 360;
  
  }

  public double degreesToEncoderTicks(double degrees) {
    double turretRotations = degrees / 360;
    return turretRotations * TurretConstants.kEncoderCPR * TurretConstants.kGearRatio;
  }

  public void setTurretDegrees(double degrees){
    double ticks = (degreesToEncoderTicks(degrees));
    m_turretMotor.set(TalonSRXControlMode.MotionMagic, ticks);
  }

  public double getTurretDegrees(){
    return encoderTicksToDegrees(m_turretMotor.getSelectedSensorPosition());
  }
    




  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

      SmartDashboard.putBoolean("target found", m_limelightData.getTargetFound());
      SmartDashboard.putNumber("skew", m_limelightData.getSkew());
      SmartDashboard.putNumber("horizontal offset", m_limelightData.getHorizontalOffset());
      SmartDashboard.putNumber("vertical offset", m_limelightData.getVerticalOffset());
      
    
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  

 

}
