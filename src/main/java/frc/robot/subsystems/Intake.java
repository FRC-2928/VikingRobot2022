// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.simulation.IntakeSim;

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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import com.revrobotics.ColorMatch;

public class Intake extends SubsystemBase {

  private Alliance m_alliance;
  private Alliance m_ballColor = Alliance.Blue;

  Solenoid m_rampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kRampSolenoid);
  // Solenoid m_rampSolenoidOpen = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kRampSolenoidOpen);
  // Solenoid m_rampSolenoidClosed = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kRampSolenoidClosed);


  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  
 
  private final WPI_TalonSRX m_rightFeederMotor = new WPI_TalonSRX(Constants.CANBusIDs.kRightFeederMotor);
  private final WPI_TalonSRX m_leftFeederMotor = new WPI_TalonSRX(Constants.CANBusIDs.kLeftFeederMotor);
  private final DigitalInput m_leftIntakeSensor = new DigitalInput(IntakeConstants.kLeftIntakeSensor);
  private final DigitalInput m_rightIntakeSensor = new DigitalInput(IntakeConstants.kRightIntakeSensor);
  
  private final TalonSRX m_intakeMotor  = new TalonSRX(Constants.CANBusIDs.kIntakeMotor);

  private boolean m_rampStable = true;
  private Timer m_timer = new Timer();
  private Timer m_colorTimer = new Timer();
  private Timer m_intakeTimer = new Timer();
  private double m_duration = 0;
  private boolean m_startIntakeTimer = true;
  private boolean m_ejectInProgress = false;

  // ------- Shuffleboard variables ----------------------------------------
  private ShuffleboardTab m_intakeTab;
  private ShuffleboardLayout m_commandsLayout;
  NetworkTableEntry m_intakeBrakeEnabledEntry, m_feederBrakeEnabledEntry;
  NetworkTableEntry m_intakeBrakeActivatedEntry, m_feederBrakeActivatedEntry;
  NetworkTableEntry m_intakeHasBallEntry, m_feederHasBallEntry;
  NetworkTableEntry m_intakeMotorEntry, m_feederMotorEntry;
  NetworkTableEntry m_rampEntry; 
  NetworkTableEntry m_allianceEntry, m_ballColorEntry, m_ballValidEntry;

  // ------ Simulation classes to help us simulate our robot ----------------
  TalonSRXSimCollection m_intakeMotorSim = m_intakeMotor.getSimCollection();
  TalonSRXSimCollection m_leftFeederMotorSim = m_leftFeederMotor.getSimCollection();
  TalonSRXSimCollection m_rightFeederMotorSim = m_rightFeederMotor.getSimCollection();
  private boolean m_rampOpenSim = false;
  private boolean m_intakeBrakeEnabled, m_feederBrakeEnabled;

  private final IntakeSim m_intakeSim = new IntakeSim(IntakeConstants.kIntakeSystem); 
  
  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Intake(Alliance alliance) {
    setAllianceColor(alliance);
    // SmartDashboard.putString("Alliance", m_alliance.name());
    // SmartDashboard.putNumber("Alliance Ordinal", m_ballColor.ordinal());
    configMotors();
    setIntakePIDF();
    resetEncoders();
    setupShuffleboard();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    // Used to regulate calls to the color sensor. Re: I2C Lockup issue
    m_colorTimer.reset();
    m_colorTimer.start();
  }

  public void configMotors(){

    
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
      srx.setNeutralMode(NeutralMode.Brake);

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

    
    m_leftFeederMotor.follow(m_rightFeederMotor, FollowerType.PercentOutput);
    m_rightFeederMotor.setInverted(true);

    //configure limit switches for intake motor and leading feeder motor
    m_rightFeederMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                                      LimitSwitchNormal.NormallyClosed, 0);
  
    setFeederBrakeEnabled();
    
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

    // Intake
    ShuffleboardLayout intakeLayout = Shuffleboard.getTab("Intake")
      .getLayout("Intake", BuiltInLayouts.kList)
      .withSize(2, 5)
      .withPosition(5, 0); 
    m_intakeBrakeEnabledEntry = intakeLayout.add("Switch Enabled", isIntakeBrakeEnabled()).getEntry();
    m_intakeBrakeActivatedEntry = intakeLayout.add("Brake Activated", isIntakeBrakeActivated()).getEntry(); 
    m_intakeMotorEntry = intakeLayout.add("Motor Speed", m_intakeMotor.getMotorOutputPercent()).getEntry(); 
    m_intakeHasBallEntry = intakeLayout.add("Has Ball", intakeHasBall()).getEntry();
  
    // Feeder
    ShuffleboardLayout feederLayout = Shuffleboard.getTab("Intake")
      .getLayout("Feeder", BuiltInLayouts.kList)
      .withSize(2, 5)
      .withPosition(7, 0); 
    m_feederBrakeEnabledEntry = feederLayout.add("Switch Enabled", isIntakeBrakeEnabled()).getEntry();
    m_feederBrakeActivatedEntry = feederLayout.add("Brake Activated", isIntakeBrakeActivated()).getEntry(); 
    m_feederMotorEntry = feederLayout.add("Motor Speed", m_rightFeederMotor.getMotorOutputPercent()).getEntry();  
    m_feederHasBallEntry = feederLayout.add("Has Ball", feederHasBall()).getEntry(); 

    // Ramp
    ShuffleboardLayout rampLayout = Shuffleboard.getTab("Intake")
      .getLayout("Ramp", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(10, 0); 
    m_rampEntry = rampLayout.add("Ramp Open?", isRampOpen()).getEntry();  

    // Ball 
    ShuffleboardLayout ballLayout = Shuffleboard.getTab("Intake")
      .getLayout("Ball", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withPosition(10, 5); 
    m_allianceEntry = ballLayout.add("Alliance", m_alliance.name()).getEntry();
    m_ballColorEntry = ballLayout.add("Ball Color", m_ballColor.name()).getEntry();  
    m_ballValidEntry = ballLayout.add("Valid Ball?", false).getEntry();

    // Commands
    m_commandsLayout = Shuffleboard.getTab("Intake")
      .getLayout("Commands", BuiltInLayouts.kList)
      .withSize(3, 6)
      .withProperties(Map.of("Label position", "HIDDEN")) // hide labels for commands
      .withPosition(1, 0);  
  }

  public void startMotors(){
    startFeederMotor(IntakeConstants.kFeederSpeed);
    startIntakeMotor(IntakeConstants.kIntakeSpeed);
  }


  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {

    if (isIntakeSensorTripped() && feederHasBall()) {
      stopIntakeMotorDelayed();
    } else {
      startIntakeMotor(Constants.IntakeConstants.kIntakeSpeed);
    }

    // Get the color of the ball that is in the feeder
    if (feederHasBall()) {
      // Regulate call frequency to the color sensor
      if (m_colorTimer.hasElapsed(0.1)) {     
        m_ballColor = getBallColor();

        m_colorTimer.reset();
        m_colorTimer.start();
      }     
    }

    publishTelemetry();
    
    // ejectBall();
    
  }

  public void publishTelemetry() {
    // Shuffleboard output
    m_intakeMotorEntry.setNumber(m_intakeMotor.getMotorOutputPercent());
    m_feederMotorEntry.setNumber(m_rightFeederMotor.getMotorOutputPercent());

    m_intakeBrakeActivatedEntry.setBoolean(isIntakeBrakeActivated());
    m_feederBrakeActivatedEntry.setBoolean(isFeederBrakeActivated());

    m_intakeHasBallEntry.setBoolean(intakeHasBall());
    m_feederHasBallEntry.setBoolean(feederHasBall());

    m_feederBrakeEnabledEntry.setBoolean(isFeederBrakeEnabled());
    m_intakeBrakeEnabledEntry.setBoolean(isIntakeBrakeEnabled());

    m_rampEntry.setBoolean(isRampOpen());
    m_ballValidEntry.setBoolean(hasValidBall());
    m_ballColorEntry.setString(m_ballColor.name());
  }

  public void ejectBall() {
    if (feederHasBall() && hasInvalidBall() && m_ejectInProgress == false) {
      m_ejectInProgress = true;
      openRamp();
      // Set the timer to wait until the ramp is open
      m_duration = 0.2;
      m_timer.reset();
      m_timer.start();
    }
  
    if (m_timer.hasElapsed(m_duration) && m_ejectInProgress) {
      if (isRampOpen() && isFeederBrakeEnabled()) {
        setFeederBrakeDisabled();
        startFeederMotor(IntakeConstants.kFeederSpeed);
        // Reset the timer to wait for the ball to leave
        m_duration = 1.0;
        m_timer.reset();
        m_timer.start();
      } else {
        closeRamp();
        setFeederBrakeEnabled();
        m_ejectInProgress = false;
      }
    }
  }

  public ShuffleboardLayout getCommandsLayout() {
    return m_commandsLayout;
  }

  public void setAllianceColor(Alliance alliance) {
    m_alliance = alliance;
  }

  public Alliance getBallColor() {

    // Ball color detection
    Color detectedColor = m_colorSensor.getColor();

    // Run the color match algorithm on our detected color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    double IR = m_colorSensor.getIR();

    Alliance ballColor;
    if (RobotBase.isReal()) {
      if (IR < 5){
        ballColor = Alliance.Invalid;
      } else if (match.color == kBlueTarget) {
        ballColor = Alliance.Blue;
      } else if (match.color == kRedTarget) {
        ballColor = Alliance.Red;
      } else {
        ballColor = Alliance.Invalid;
      }
    } else {
      ballColor = m_intakeSim.getBallColor();
    }
  
    
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Detected Color Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", ballColor.name());
    SmartDashboard.putNumber("Detected Red", detectedColor.red);
    SmartDashboard.putNumber("Detected Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    

    return ballColor;
  }

  // --------- Intake Motor ------------------------------
  public void stopIntakeMotor(){
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Delays the stopping of the intake motor for 0.1 seconds
   * This must be run in a periodic loop.
   */
  public void stopIntakeMotorDelayed(){
    if (m_startIntakeTimer) {
      m_intakeTimer.reset();
      m_intakeTimer.start();
      m_startIntakeTimer = false;
    }     

    if (m_intakeTimer.hasElapsed(0.1)) {
      stopIntakeMotor();
      m_startIntakeTimer = true;
    }    
  }

  /**
   * 
   * @param output the output of the motor, between -1 and 1
   */
  public void startIntakeMotor(double output){
    m_intakeMotor.set(ControlMode.PercentOutput, output);
    setIntakeBrakeOverride();
  }

  /**
   * sets the intake brake enabled if the feeder has a ball, and disabled if not
   */
  public void setIntakeBrakeOverride(){
    if(feederHasBall()){
      setIntakeBrakeEnabled();
    } else {
      setIntakeBrakeDisabled();
    }
  }

  /**
   * listen to intake limit switch
   */
  public void setIntakeBrakeEnabled(){
    m_intakeMotor.overrideLimitSwitchesEnable(false);
    m_intakeBrakeEnabled = true;
  }

  /**
   * ignore intake limit switch - motor keeps running
   */
  public void setIntakeBrakeDisabled(){
    m_intakeMotor.overrideLimitSwitchesEnable(true);
    m_intakeBrakeEnabled = false;
    if (!RobotBase.isReal()) {
      m_intakeSim.triggerOpenIntakeSwitchSim();
    }
  }

  // --------- Feeder Motor ------------------------------
  public void stopFeederMotor(){
    m_rightFeederMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * 
   * @param output the output of the motor, between -1 and 1
   */
  public void startFeederMotor(double output){
    m_rightFeederMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * listen to feeder limit switch
   */
  public void setFeederBrakeEnabled(){
    //overriding the brake (true) means the brake is disabled
    m_rightFeederMotor.overrideLimitSwitchesEnable(true);
    m_feederBrakeEnabled = true;
    if (!RobotBase.isReal()) {
      m_intakeSim.triggerOpenFeederSwitchSim();
    }
  }

  /**
   * ignore feeder limit switch
   */
  public void setFeederBrakeDisabled(){
    m_rightFeederMotor.overrideLimitSwitchesEnable(false);
    m_feederBrakeEnabled = false;
  }

  // --------- Ramp ------------------------------

  //TODO: maybe switch false and true depending on which solenoid state is the open ramp
  public void openRamp(){
    m_rampSolenoid.set(true);

    //for double solenoids on sweetpants
    // m_rampSolenoidOpen.set(true);
    // m_rampSolenoidClosed.set(false);

    m_rampOpenSim = true;
  }

  public void closeRamp(){
    m_rampSolenoid.set(false);

    //for double solenoids on sweetpants
    // m_rampSolenoidOpen.set(false);
    // m_rampSolenoidClosed.set(true);

    m_rampOpenSim = false;
  }

  public void setRampState(boolean state){
    m_rampStable = state;
  }

  public void setRampStable() {
    m_rampStable = true;
  }

  public void setRampUnstable() {
    m_rampStable = false;
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  public boolean readyToShoot() {
    if (isRampClosed() && hasValidBall()) {
      return true;
    } 
    return false;
  }

  public boolean hasValidBall() {
    return (m_alliance.ordinal() == m_ballColor.ordinal());
  }

  public boolean hasInvalidBall() {
    return !hasValidBall();
  }

  // --------- Intake ------------------------------  

  public boolean isIntakeBrakeActivated() {
    // return isIntakeSwitchActivated() && isIntakeBrakeEnabled();
    return isIntakeSensorTripped();
  }

  public boolean isIntakeSwitchActivated() {
    // Simulate this return if not running on the real robot
    if (RobotBase.isReal()) {
      return m_intakeMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }
    return m_intakeSim.isIntakeSwitchClosed();
  }

  public boolean intakeHasBall(){
    return isIntakeBrakeActivated();
  }

  public boolean intakeCleared() {
    return !intakeHasBall();
  }


  public boolean isIntakeMotorOn(){
    //not percent, range between -1 and 1
    return (m_intakeMotor.getMotorOutputPercent() > .1);
  }

  public boolean isIntakeBrakeEnabled() {
    return m_intakeBrakeEnabled;
  }

  public boolean isLeftIntakeSensorTripped(){
    return !m_leftIntakeSensor.get();
  }

  public boolean isRightIntakeSensorTripped(){
    return !m_rightIntakeSensor.get();
  }

  public boolean isIntakeSensorTripped() {
    return isLeftIntakeSensorTripped() || isRightIntakeSensorTripped();
  }


  // --------- Feeder ------------------------------

  public boolean isFeederBrakeActivated(){
    return isFeederSwitchActivated();
  }

  public boolean isFeederSwitchActivated() {
    // Simulate this return if not running on the real robot
    if (RobotBase.isReal()) {
      return !(m_rightFeederMotor.getSensorCollection().isFwdLimitSwitchClosed());
      // return !(m_rightFeederMotor.getSensorCollection().isRevLimitSwitchClosed());
    }
    return m_intakeSim.isFeederSwitchClosed();
  }

  public boolean isFeederBrakeDeactivated(){
    return !(isFeederBrakeActivated());
  }
   
  public boolean feederHasBall(){
    return isFeederBrakeActivated();  
  }

  public boolean feederCleared() {
    return !feederHasBall();
  }

  public boolean isFeederMotorOn(){
    //not percent, range between -1 and 1
    return (Math.abs(m_rightFeederMotor.getMotorOutputPercent()) > .1);
  }  

  public boolean isFeederBrakeEnabled() {
    return m_feederBrakeEnabled;
  }

  // --------- Ramp ------------------------------

  //TODO: maybe switch true to false depending on which solenoid state is the open ramp
  public boolean isRampOpen(){
    // Simulate this return if not running on the real robot
    // if (RobotBase.isReal()) {
    //   return (m_rampSolenoid.get());
    // }  
    return m_rampOpenSim;
  }

  public boolean isRampClosed(){
    return !(isRampOpen());
  }

  // -----------------------------------------------------------
  // Simulation
  // -----------------------------------------------------------
  
  public void simulationPeriodic() {

    if (m_intakeSim.isIntakeSwitchClosed()) {
      if (feederHasBall()) {
        m_intakeSim.triggerCloseIntakeSwitchSim();
      }
      else 
      { m_intakeSim.triggerCloseFeederSwitchSim(); 
        m_intakeSim.triggerOpenIntakeSwitchSim();         
      }          
    }

    /* Pass the robot battery voltage to the simulated Talon SRXs */
    // If the brake is activated we simulate the fact that the motor has stopped.
    if (isIntakeBrakeActivated()) {
      m_intakeMotorSim.setBusVoltage(0);
    } else {
      m_intakeMotorSim.setBusVoltage(RobotController.getInputVoltage());
    }

    if (isFeederBrakeActivated()) {
      m_rightFeederMotorSim.setBusVoltage(0);
    } else {
      m_rightFeederMotorSim.setBusVoltage(RobotController.getInputVoltage());
    }
    m_leftFeederMotorSim.setBusVoltage(RobotController.getInputVoltage());
    
    // In this method, we update our simulation of what our intake is doing.
    // First, we set our "inputs" (voltages)
    m_intakeSim.setInput(m_intakeMotorSim.getMotorOutputLeadVoltage());
    m_intakeSim.setInput(m_rightFeederMotorSim.getMotorOutputLeadVoltage());
    
    // Next, we update it. The standard loop time is 20ms.
    m_intakeSim.update(0.02);

    // Finally, we set our simulated encoder's readings
    m_intakeMotorSim.setQuadratureRawPosition((int)m_intakeSim.getOutput(0));
    m_rightFeederMotorSim.setQuadratureRawPosition((int)m_intakeSim.getOutput(0));
  }  
  
  public void triggerCloseIntakeSwitchSim() {
    m_intakeSim.triggerCloseIntakeSwitchSim();
  }

}

