// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants;


public class Flywheel extends SubsystemBase {

  private final WPI_TalonFX m_flywheelTalon = new WPI_TalonFX(Constants.CANBusIDs.kFlywheelTalonFX);
  double m_velocity;
  double m_adjustableVelocity;
  double m_velocityChange;
  boolean m_isFlywheelSpunUp = false;

  //simulation
  TalonFXSimCollection m_flywheelMotorSim = m_flywheelTalon.getSimCollection();

  ShuffleboardLayout m_flywheelLayout;
  NetworkTableEntry m_flywheelSpeedEntry;
  NetworkTableEntry m_flywheelPercentEntry;
  NetworkTableEntry m_flywheelTicksEntry;
  private ShuffleboardLayout m_commandsLayout;

  Timer m_shootTimer = new Timer();

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  /** Creates a new Flywheel. */
  public Flywheel() {
    configMotors();
    resetEncoders();
    setFlywheelPIDF();
    m_velocityChange = 1;
    DistanceMap.getInstance().loadMaps();
    m_adjustableVelocity = FlywheelConstants.kIdealVelocity;
    m_shootTimer.reset();
    m_shootTimer.start();
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

    m_flywheelTalon.configOpenloopRamp(0.2);
  

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

    m_flywheelTalon.setInverted(true);
  }

  public void setFlywheelPIDF() {
    m_flywheelTalon.config_kP(0, FlywheelConstants.kGainsVelocity1.kP, 0);
    m_flywheelTalon.config_kI(0, FlywheelConstants.kGainsVelocity1.kI, 0);
    m_flywheelTalon.config_kD(0, FlywheelConstants.kGainsVelocity1.kD, 0);
    m_flywheelTalon.config_kF(0, FlywheelConstants.kGainsVelocity1.kF, 0);

    m_flywheelTalon.config_kP(1, FlywheelConstants.kGainsVelocity2.kP, 0);
    m_flywheelTalon.config_kI(1, FlywheelConstants.kGainsVelocity2.kI, 0);
    m_flywheelTalon.config_kD(1, FlywheelConstants.kGainsVelocity2.kD, 0);
    m_flywheelTalon.config_kF(1, FlywheelConstants.kGainsVelocity2.kF, 0);

    m_flywheelTalon.config_kP(2, FlywheelConstants.kGainsVelocity3.kP, 0);
    m_flywheelTalon.config_kI(2, FlywheelConstants.kGainsVelocity3.kI, 0);
    m_flywheelTalon.config_kD(2, FlywheelConstants.kGainsVelocity3.kD, 0);
    m_flywheelTalon.config_kF(2, FlywheelConstants.kGainsVelocity3.kF, 0);

    m_flywheelTalon.config_kP(3, FlywheelConstants.kGainsVelocity4.kP, 0);
    m_flywheelTalon.config_kI(3, FlywheelConstants.kGainsVelocity4.kI, 0);
    m_flywheelTalon.config_kD(3, FlywheelConstants.kGainsVelocity4.kD, 0);
    m_flywheelTalon.config_kF(3, FlywheelConstants.kGainsVelocity4.kF, 0);

  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  @Override
  public void periodic() {   
    publishTelemetry();
    if(m_shootTimer.hasElapsed(2)){
    m_shootTimer.reset();
    m_shootTimer.start();
    }
  }

  public void publishTelemetry() {

    SmartDashboard.putNumber("Flywheel Speed", m_flywheelTalon.getSelectedSensorVelocity());

    m_flywheelPercentEntry.setNumber(m_flywheelTalon.getMotorOutputPercent());
    m_flywheelSpeedEntry.setNumber(m_flywheelTalon.getSelectedSensorVelocity());
    m_flywheelTicksEntry.setNumber(m_flywheelTalon.getSelectedSensorVelocity());
  }

  public void resetEncoders(){
    m_flywheelTalon.setSelectedSensorPosition(0);
  }

  /**
   * Set the PID values based on the required flywheel speed.
   * 
   * @param flywheelTicksPer100ms flywheel speed in tick per/100ms
   */
  public void setPidGains(int flywheelTicksPer100ms) {
    if(flywheelTicksPer100ms > 16000){
      m_flywheelTalon.selectProfileSlot(3, 0);
    } else if (flywheelTicksPer100ms > 13000) {
      m_flywheelTalon.selectProfileSlot(1, 0);
    } else if (flywheelTicksPer100ms > 8500) {
      m_flywheelTalon.selectProfileSlot(0, 0);
    } else {
      m_flywheelTalon.selectProfileSlot(2, 0);
    }
  }


  // ---------------- Velocity --------------------

  /**
   * sets velocity to whatever is stored as the velocity variable, multiplied by the change in velocity
   */
  public void setVelocity(){

    //sets as ticks per 100 ms
    if(m_adjustableVelocity * m_velocityChange >= 20000){
      m_flywheelTalon.set(ControlMode.Velocity, 20000);
    } else {
      m_flywheelTalon.set(ControlMode.Velocity, m_adjustableVelocity * m_velocityChange);
    }
  }
  
  public void turnFlywheelOff() {
    m_flywheelTalon.set(ControlMode.PercentOutput, 0);
  }

  /**
   * 
   * @param velocity resets the velocity for flywheel to go to
   */
  public void setAdjustableVelocity(double velocity){
    m_adjustableVelocity = velocity;
  }

  public void increaseFlywheelChange(){
    m_velocityChange += .02;
  }

  public void decreaseFlywheelChange(){
    m_velocityChange -= .02;
  }


  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  /**
   * 
   * @return velocity in ticks per 100ms
   */
  public double getVelocity(){
    return (m_flywheelTalon.getSelectedSensorVelocity()); 
  }

  public boolean isFlywheelMotorOn(){
    return(m_flywheelTalon.getMotorOutputPercent() > 0);
  }

  /**
   * 
   * @return whether flywheel is within 200 ticks/100ms of desired velocity
   */
  public boolean isFlyWheelUpToSpeed(){

    if(Math.abs((m_adjustableVelocity * m_velocityChange) - getVelocity()) <= 200.0){
      return true;
    }else{
      return false;
    }
  }
}
