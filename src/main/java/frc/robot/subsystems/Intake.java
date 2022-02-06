// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightData;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  Solenoid m_rampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.kRampSolenoid);
 
  private final WPI_TalonSRX m_rightRampMotor = new WPI_TalonSRX(RobotMap.kRightRampMotor);
  private final WPI_TalonSRX m_leftRampMotor = new WPI_TalonSRX(RobotMap.kLeftRampMotor);
  
  private final TalonSRX m_intakeMotor  = new TalonSRX(Constants.RobotMap.kIntakeMotor);


  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Intake() {
    configMotors();
    setIntakePIDF();
    resetEncoders();
    
  }

  public void configMotors(){

    m_leftRampMotor.follow(m_rightRampMotor, FollowerType.PercentOutput);

    //configure limit switches for intake motor and leading feeder motor
    m_intakeMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
      LimitSwitchNormal.NormallyOpen, 0);
    m_rightRampMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
      LimitSwitchNormal.NormallyOpen, 0);
    
    for(TalonSRX srx : new TalonSRX[] {m_intakeMotor, m_rightRampMotor, m_leftRampMotor}) {
    
      //Reset settings for safety
      srx.configFactoryDefault();

      //Sets voltage compensation to 12, used for percent output
      srx.configVoltageCompSaturation(10);
      srx.enableVoltageCompensation(true);

      //Setting just in case
      srx.configNominalOutputForward(0);
      srx.configNominalOutputReverse(0);
      srx.configPeakOutputForward(1);
      srx.configPeakOutputReverse(-1);

      srx.configOpenloopRamp(0.1);

      //Setting deadband(area required to start moving the motor) to 1%
      srx.configNeutralDeadband(0.01);

      //Set to brake mode, will brake the motor when no power is sent
      srx.setNeutralMode(NeutralMode.Coast);

      /** 
       * Setting input side current limit (amps)
       * 45 continious, 80 peak, 30 millieseconds allowed at peak
       * 40 amp breaker can support above 40 amps for a little bit
       * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
       */
      srx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

      //Either using the integrated Falcon sensor or an external one, will change if needed
      srx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 

    }
  }

  public void setIntakePIDF() {
    m_intakeMotor.config_kP(0, IntakeConstants.IntakekP, 0);
    m_intakeMotor.config_kI(0, IntakeConstants.IntakekI, 0);
    m_intakeMotor.config_kD(0, IntakeConstants.IntakekD, 0);
    m_intakeMotor.config_kF(0, IntakeConstants.IntakekF, 0);
  }

  public void resetEncoders(){
    m_intakeMotor.setSelectedSensorPosition(0);
  }

  public void setupShuffleboard() {
    ShuffleboardTab m_intakeTab = Shuffleboard.getTab("Intake");  
  }


  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void stopIntakeMotor(){

    m_intakeMotor.set(ControlMode.PercentOutput, 0);

  }

  /**
   * 
   * @param output the percent output of the motor, between -1 and 1
   */
  public void startIntakeMotor(double output){

    m_intakeMotor.set(ControlMode.PercentOutput, output);

  }

  public void stopFeederMotor(){

    m_rightRampMotor.set(ControlMode.PercentOutput, 0);

  }

  /**
   * 
   * @param output the percent output of the motor, between -1 and 1
   */
  public void startFeederMotor(double output){

    m_rightRampMotor.set(ControlMode.PercentOutput, output);

  }

  public void setFeederBrakeOff(){


  }

  /**
   * 
   * @param override true if the limit switch is being overriden
   */
  public void overrideFeederBreak(boolean override){
    m_rightRampMotor.overrideLimitSwitchesEnable(override);
  }

  /**
   * 
   * @param override true if the limit switch is being overriden
   */
  public void overrideIntakeBreak(boolean override){
    m_intakeMotor.overrideLimitSwitchesEnable(override);
  }

  //TODO: maybe switch false and true depending on which solenoid state is the open ramp
  public void openRamp(){
    m_rampSolenoid.set(true);
  }

  public void closeRamp(){
    m_rampSolenoid.set(false);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  public boolean isFeederBrakeOn(){
    //clear the talon to see if brake is on
    return true;
    
  }
   
  public boolean isFeederClear(){
    return true;

  }

  public boolean isBallOnRamp(){


    return true;

  }

  public boolean isBallOnDeck(){

    return true;


  }

  public boolean isIntakeArmUp(){

    return true;

  }

  //TODO: maybe switch true to false depending on which solenoid state is the open ramp
  public boolean isRampOpen(){
    return (m_rampSolenoid.get());
  }

  
  
}

