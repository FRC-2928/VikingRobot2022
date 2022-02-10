// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightData;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.IntakeCommands.ToggleIntakeMotor;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  Solenoid m_rampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.kRampSolenoid);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  
 
  private final WPI_TalonSRX m_rightFeederMotor = new WPI_TalonSRX(RobotMap.kRightRampMotor);
  private final WPI_TalonSRX m_leftFeederMotor = new WPI_TalonSRX(RobotMap.kLeftRampMotor);
  
  private final TalonSRX m_intakeMotor  = new TalonSRX(Constants.RobotMap.kIntakeMotor);

  private boolean m_isFeederClear;
  private boolean m_rampstable = true;
  private boolean m_intakeBrakeEnabled, m_feederBrakeEnabled;
  private double m_last_timestamp;

  // Setup Shuffleboard 
  private ShuffleboardTab m_intakeTab;
  private ShuffleboardLayout m_commandsLayout;
  NetworkTableEntry m_intakeBrakeEnabledEntry, m_feederBrakeEnabledEntry;
  NetworkTableEntry m_intakeBrakeActivatedEntry, m_feederBrakeActivatedEntry;
  NetworkTableEntry m_intakeHasBallEntry, m_feederHasBallEntry;
  NetworkTableEntry m_intakeMotorEntry, m_feederMotorEntry;

  // ------ Simulation classes to help us simulate our robot ---------
  TalonSRXSimCollection m_intakeMotorSim = m_intakeMotor.getSimCollection();
  TalonSRXSimCollection m_leftFeederMotorSim = m_leftFeederMotor.getSimCollection();
  TalonSRXSimCollection m_rightFeederMotorSim = m_rightFeederMotor.getSimCollection();
  private boolean m_intakeActivatedSim = false;
  private boolean m_feederActivatedSim = false;
  private int m_cycles = 0;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Intake() {
    configMotors();
    setIntakePIDF();
    resetEncoders();
    setupShuffleboard();
    m_isFeederClear = false;
    // setFeederBrakeEnabled();
    // setIntakeBrakeDisabled();
  }

  public void configMotors(){

    m_leftFeederMotor.follow(m_rightFeederMotor, FollowerType.PercentOutput);

    //configure limit switches for intake motor and leading feeder motor
    m_intakeMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
      LimitSwitchNormal.NormallyOpen, 0);
    m_rightFeederMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
      LimitSwitchNormal.NormallyOpen, 0);
    
    for(TalonSRX srx : new TalonSRX[] {m_intakeMotor, m_rightFeederMotor, m_leftFeederMotor}) {
    
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
    m_intakeTab = Shuffleboard.getTab("Intake"); 

    ShuffleboardLayout intakeLayout = Shuffleboard.getTab("Intake")
      .getLayout("Intake", BuiltInLayouts.kList)
      .withSize(1, 3)
      .withPosition(5, 0); 

    m_intakeBrakeEnabledEntry = intakeLayout.add("Brake Enabled", isIntakeBrakeEnabled()).getEntry();
    m_intakeBrakeActivatedEntry = intakeLayout.add("Brake Activated", isIntakeBrakeActivated()).getEntry(); 
    m_intakeMotorEntry = intakeLayout.add("Motor Speed", m_intakeMotor.getSelectedSensorVelocity()).getEntry(); 
    m_intakeHasBallEntry = intakeLayout.add("Has Ball", intakeHasBall()).getEntry();   
  
    ShuffleboardLayout feederLayout = Shuffleboard.getTab("Intake")
      .getLayout("Feeder", BuiltInLayouts.kList)
      .withSize(1, 3)
      .withPosition(6, 0); 

    m_feederBrakeEnabledEntry = feederLayout.add("Brake Enabled", isIntakeBrakeEnabled()).getEntry();
    m_feederBrakeActivatedEntry = feederLayout.add("Brake Activated", isIntakeBrakeActivated()).getEntry(); 
    m_feederMotorEntry = feederLayout.add("Motor Speed", m_rightFeederMotor.getSelectedSensorVelocity()).getEntry();  
    m_feederHasBallEntry = feederLayout.add("Has Ball", feederHasBall()).getEntry(); 

    m_commandsLayout = Shuffleboard.getTab("Intake")
      .getLayout("Commands", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withProperties(Map.of("Label position", "HIDDEN")) // hide labels for commands
      .withPosition(2, 0);
 
  }


  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setIntakeBrakeState();

    // Shuffleboard output
    m_intakeMotorEntry.setNumber(m_intakeMotor.getSelectedSensorVelocity());
    m_feederMotorEntry.setNumber(m_rightFeederMotor.getSelectedSensorVelocity());
    m_intakeBrakeActivatedEntry.setBoolean(isIntakeBrakeActivated());
    m_feederBrakeActivatedEntry.setBoolean(isFeederBrakeActivated());
    m_intakeHasBallEntry.setBoolean(intakeHasBall());
    m_feederHasBallEntry.setBoolean(feederHasBall());

    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("Intake Motor Voltage", m_intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Intake Motor Percent", m_intakeMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Feeder Motor Voltage", m_rightFeederMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Feeder Motor Percent", m_rightFeederMotor.getMotorOutputPercent());
    
  }

  public ShuffleboardLayout getCommandsLayout() {
    return m_commandsLayout;
  }

  public void setIntakeBrakeState() {
    if (feederHasBall()) {
      setIntakeBrakeEnabled();
    } else {
      setIntakeBrakeDisabled();
    }
  }

  public void stopIntakeMotor(){
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
    setIntakeBrakeDisabled();
  }

  public void startMotors() {
    startFeederMotor(IntakeConstants.kFeederDefaultSpeed);
    startIntakeMotor(IntakeConstants.kIntakeDefaultSpeed);
  }

  /**
   * 
   * @param output the percent output of the motor, between -1 and 1
   */
  public void startIntakeMotor(double output){
    setFeederBrakeEnabled();
    m_intakeMotor.set(ControlMode.PercentOutput, output);
  }

  public void stopFeederMotor(){
    m_rightFeederMotor.set(ControlMode.PercentOutput, 0);
    setFeederBrakeDisabled();
  }

  /**
   * 
   * @param output the percent output of the motor, between -1 and 1
   */
  public void startFeederMotor(double output){
    System.out.println("Starting Feeder motor...");
    setIntakeBrakeState();
    m_rightFeederMotor.set(ControlMode.PercentOutput, output);
  }

  public void setFeederBrakeDisabled(){
    m_rightFeederMotor.overrideLimitSwitchesEnable(false);
    m_feederBrakeEnabled = false;
    m_feederBrakeEnabledEntry.setBoolean(m_feederBrakeEnabled);
  }

  public void setFeederBrakeEnabled(){
    m_rightFeederMotor.overrideLimitSwitchesEnable(true);
    m_feederBrakeEnabled = true;
    m_feederBrakeEnabledEntry.setBoolean(m_feederBrakeEnabled);
  }

  
  public void setIntakeBrakeEnabled(){
    m_intakeMotor.overrideLimitSwitchesEnable(true);
    m_intakeBrakeEnabled = true;
    m_intakeBrakeEnabledEntry.setBoolean(m_intakeBrakeEnabled);
  }

  public void setIntakeBrakeDisabled(){
    m_intakeMotor.overrideLimitSwitchesEnable(false);
    m_intakeBrakeEnabled = false;
    m_intakeBrakeEnabledEntry.setBoolean(m_intakeBrakeEnabled);
  }

  //TODO: maybe switch false and true depending on which solenoid state is the open ramp
  public void openRamp(){
    m_rampSolenoid.set(true);
  }

  public void closeRamp(){
    m_rampSolenoid.set(false);
  }

  public void setFeederCleared(){
    m_isFeederClear = true;
  }

  public void setRampState(boolean state){
    m_rampstable = state;

  }
  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  public boolean readyToShoot() {
    if (feederHasBall() && isRampClosed()) {
      return true;
    } 
    return false;
  }

  public boolean isFeederBrakeActivated(){
    if (RobotBase.isReal()) {
      return m_rightFeederMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }
    return m_feederActivatedSim; 
  }

  public boolean isFeederBrakeDeactivated(){
    return !(isFeederBrakeActivated());
  }
   
  public boolean isIntakeBrakeActivated() {
    if (RobotBase.isReal()) {
      return m_intakeMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }
    return m_intakeActivatedSim;
  }

  public boolean isIntakeBrakeEnabled() {
    return m_intakeBrakeEnabled;
  }

  public boolean isFeederBrakeEnabled() {
    return m_feederBrakeEnabled;
  }

  public boolean isFeederClear(){    
    return m_isFeederClear;
  }

  public boolean intakeHasBall(){
    return isIntakeArmUp();
  }

  public boolean feederHasBall(){
    return isFeederBrakeActivated();  
  }

  public boolean isIntakeArmUp(){
    return isIntakeBrakeActivated();
  }

  //TODO: maybe switch true to false depending on which solenoid state is the open ramp
  public boolean isRampOpen(){
    return (m_rampSolenoid.get());
  }

  public boolean isRampClosed(){
    return !(isRampOpen());
  }

  public boolean isIntakeMotorOn(){
   return (m_intakeMotor.getMotorOutputPercent() > 10);
  }

  public boolean isFeederMotorOn(){
    return (m_rightFeederMotor.getMotorOutputPercent() > 10);
   }

  // -----------------------------------------------------------
  // Simulation
  // -----------------------------------------------------------
  public void triggerIntakeBrakeActivatedSim() {
    m_intakeActivatedSim = true;
    m_cycles = 0;
  }

  public void triggerIntakeBrakeDeactivatedSim() {
    m_intakeActivatedSim = false;
  }

  public void triggerFeederBrakeActivatedSim() {
    m_feederActivatedSim = true;
  }

  public void simulationPeriodic() {

    if (isIntakeBrakeActivated()) {
      triggerFeederBrakeActivatedSim();  
      m_cycles += 1;
    }

    if (m_cycles > 20) {
      triggerIntakeBrakeDeactivatedSim();
    };
    

    /* Pass the robot battery voltage to the simulated Talon SRXs */
    m_intakeMotorSim.setBusVoltage(RobotController.getInputVoltage());
    m_leftFeederMotorSim.setBusVoltage(RobotController.getInputVoltage());
    m_rightFeederMotorSim.setBusVoltage(RobotController.getInputVoltage());

    
  }  
  
}

