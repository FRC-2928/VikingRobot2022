// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMap;

public class Climber extends SubsystemBase {

  private final WPI_TalonSRX m_climberMotor = new WPI_TalonSRX(RobotMap.kClimberMotor);

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  
  /** Creates a new Climber. */
  public Climber() {
    configMotors();
    resetEncoders();
    setupShuffleboard();
    setClimberPIDF();
  }


  public void configMotors(){

    //Reset settings for safety
    m_climberMotor.configFactoryDefault();

    //Sets voltage compensation to 12, used for percent output
    m_climberMotor.configVoltageCompSaturation(10);
    m_climberMotor.enableVoltageCompensation(true);

    //Setting just in case
    m_climberMotor.configNominalOutputForward(0);
    m_climberMotor.configNominalOutputReverse(0);
    m_climberMotor.configPeakOutputForward(1);
    m_climberMotor.configPeakOutputReverse(-1);

    m_climberMotor.configOpenloopRamp(0.1);

    //Setting deadband(area required to start moving the motor) to 1%
    m_climberMotor.configNeutralDeadband(0.01);

    //Set to brake mode, will brake the motor when no power is sent
    m_climberMotor.setNeutralMode(NeutralMode.Coast);

    /** 
     * Setting input side current limit (amps)
     * 45 continious, 80 peak, 30 millieseconds allowed at peak
     * 40 amp breaker can support above 40 amps for a little bit
     * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
     */
    m_climberMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

    //Either using the integrated Falcon sensor or an external one, will change if needed
    m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
  }

  public void setClimberPIDF() {
    m_climberMotor.config_kP(0, ClimberConstants.kGainsClimber.kP , 0);
    m_climberMotor.config_kI(0, ClimberConstants.kGainsClimber.kI, 0);
    m_climberMotor.config_kD(0, ClimberConstants.kGainsClimber.kD, 0);
    m_climberMotor.config_kF(0, ClimberConstants.kGainsClimber.kF, 0);
  }

  public void setupShuffleboard() {
    ShuffleboardTab m_climberTab = Shuffleboard.getTab("Climber");  
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders(){
    m_climberMotor.setSelectedSensorPosition(0);
  }

  /**
   * 
   * @param power percent power of the motor, between -1 and 1
   */
  public void setPower(double power){
    m_climberMotor.set(ControlMode.PercentOutput, power);
  }


  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  public double getPositionMeters(){
    double ticks = m_climberMotor.getSelectedSensorPosition();
    return (ticksToMeters(ticks));
  }

  //TODO: add whatever other step is in here, depending on mechanics....
  public double MetersToTicks(double meters){
    return ((meters * ClimberConstants.kEncoderCPR) * ClimberConstants.gearRatio);
  }

  public double ticksToMeters(double ticks){
    return ((ticks / ClimberConstants.kEncoderCPR) / ClimberConstants.gearRatio);
  }
  
}