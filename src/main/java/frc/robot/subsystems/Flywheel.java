// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TurretConstants;

public class Flywheel extends SubsystemBase {

  private final WPI_TalonFX m_flywheelTalon = new WPI_TalonFX(RobotMap.kFlywheelTalonFX);

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  /** Creates a new Flywheel. */
  public Flywheel() {
    configMotors();
    resetEncoders();
    setFlywheelPIDF();
    setupShuffleboard();
  }

  public void configMotors(){
    //Reset settings for safety
    m_flywheelTalon.configFactoryDefault();

    //Sets voltage compensation to 12, used for percent output
    m_flywheelTalon.configVoltageCompSaturation(10);
    m_flywheelTalon.enableVoltageCompensation(true);

    //Setting just in case
    m_flywheelTalon.configNominalOutputForward(0);
    m_flywheelTalon.configNominalOutputReverse(0);
    m_flywheelTalon.configPeakOutputForward(1);
    m_flywheelTalon.configPeakOutputReverse(-1);

    m_flywheelTalon.configOpenloopRamp(0.1);

    //Setting deadband(area required to start moving the motor) to 1%
    m_flywheelTalon.configNeutralDeadband(0.01);

    //Set to brake mode, will brake the motor when no power is sent
    m_flywheelTalon.setNeutralMode(NeutralMode.Coast);

    /** 
     * Setting input side current limit (amps)
     * 45 continious, 80 peak, 30 millieseconds allowed at peak
     * 40 amp breaker can support above 40 amps for a little bit
     * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
     */
    m_flywheelTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

    //Either using the integrated Falcon sensor or an external one, will change if needed
    m_flywheelTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
  }

  public void setFlywheelPIDF() {
    m_flywheelTalon.config_kP(0, FlywheelConstants.kGainsFlywheel.kP, 0);
    m_flywheelTalon.config_kI(0, FlywheelConstants.kGainsFlywheel.kI, 0);
    m_flywheelTalon.config_kD(0, FlywheelConstants.kGainsFlywheel.kD, 0);
    m_flywheelTalon.config_kF(0, FlywheelConstants.kGainsFlywheel.kF, 0);
  }

  public void setupShuffleboard() {
    ShuffleboardTab m_flywheelTab = Shuffleboard.getTab("Flywheel");  
  }


  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders(){
    m_flywheelTalon.setSelectedSensorPosition(0);
  }

  /**
   * 
   * @param power the percent power value between -1 and 1
   */
  public void setPower(double power){
    m_flywheelTalon.set(ControlMode.PercentOutput, power);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public double getRPM(){
    double ticksPerSec = m_flywheelTalon.getSelectedSensorVelocity() * 10;
    return (ticksToRotations(ticksPerSec) * 60);
  }

  public double rotationsToTicks(double rotations){
    return ((rotations * FlywheelConstants.kEncoderCPR) * FlywheelConstants.kGearRatio);
  }

  public double ticksToRotations(double ticks){
    return ((ticks / FlywheelConstants.kEncoderCPR) / FlywheelConstants.kGearRatio);
  }

  /**
   * 
   * @return the percent output of the motor (-100 to 100)
   */
  public double getPower(){
    return(m_flywheelTalon.getMotorOutputPercent());
  }

  public boolean isFlywheelMotorOn(){
    return(m_flywheelTalon.getMotorOutputPercent() > 10);
  }
  

}
