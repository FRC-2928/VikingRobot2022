// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
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
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Intake extends SubsystemBase {


  Solenoid m_rampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kRampSolenoid);
  
  private final Drivetrain m_drivetrain;
 
  private final WPI_TalonSRX m_rightFeederMotor = new WPI_TalonSRX(Constants.CANBusIDs.kRightFeederMotor);
  private final WPI_TalonSRX m_leftFeederMotor = new WPI_TalonSRX(Constants.CANBusIDs.kLeftFeederMotor);
  // private final DigitalInput m_leftIntakeSensor = new DigitalInput(IntakeConstants.kLeftIntakeSensor);
  // private final DigitalInput m_rightIntakeSensor = new DigitalInput(IntakeConstants.kRightIntakeSensor);
  
  private final TalonSRX m_intakeMotor  = new TalonSRX(Constants.CANBusIDs.kIntakeMotor);

  private Timer m_ejectTimer = new Timer();
  private Timer m_intakeTimer = new Timer();
  private Timer m_motorLowSpeedTimer = new Timer();
  private double m_ejectDuration = 0;
  private double m_intakeMotorSpeed = IntakeConstants.kIntakeSpeed;

  private boolean m_startIntakeTimer = true;
  private boolean m_ejectInProgress = false;
  private boolean m_intakeMotorStop = false;
  private boolean m_rampStable = true;
  

  // ------- Shuffleboard variables ----------------------------------------
  private ShuffleboardTab m_intakeTab;
  private ShuffleboardLayout m_commandsLayout;
  // NetworkTableEntry m_intakeBrakeEnabledEntry;
  NetworkTableEntry m_feederBrakeEnabledEntry;
  NetworkTableEntry m_intakeBrakeActivatedEntry;
  NetworkTableEntry m_feederMotorOnEntry, m_intakeMotorOnEntry;
  NetworkTableEntry m_intakeHasBallEntry, m_feederHasBallEntry;
  NetworkTableEntry m_intakeMotorEntry, m_feederMotorEntry;
  NetworkTableEntry m_rampEntry; 
  NetworkTableEntry m_allianceEntry, m_ballColorEntry, m_ballValidEntry;
  NetworkTableEntry m_intakeMotorSpeedEntry;

  // ------ Simulation classes to help us simulate our robot ----------------
  TalonSRXSimCollection m_intakeMotorSim = m_intakeMotor.getSimCollection();
  TalonSRXSimCollection m_leftFeederMotorSim = m_leftFeederMotor.getSimCollection();
  TalonSRXSimCollection m_rightFeederMotorSim = m_rightFeederMotor.getSimCollection();
  private boolean m_rampOpenSim = false;
  // private boolean m_intakeBrakeEnabled;
  private boolean m_feederBrakeEnabled;

  private final IntakeSim m_intakeSim = new IntakeSim(IntakeConstants.kIntakeSystem); 
  
  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Intake(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    configMotors();
    setIntakePIDF();
    resetEncoders();
    setupShuffleboard();

    // m_colorMatcher.addColorMatch(kBlueTarget);
    // m_colorMatcher.addColorMatch(kRedTarget);
    // Used to regulate calls to the color sensor. Re: I2C Lockup issue
    // m_colorTimer.reset();
    // m_colorTimer.start();
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

      srx.configOpenloopRamp(0);

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
    
    m_intakeMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                                  LimitSwitchNormal.NormallyOpen, 0);

  
    setFeederBrakeEnabled();
    setIntakeBrakeEnabled();
    
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
    m_intakeMotorSpeedEntry = m_intakeTab.add("Intake Motor Speed", IntakeConstants.kIntakeSpeed)
      .withPosition(3, 5)
      .getEntry();  
    ShuffleboardLayout intakeLayout = Shuffleboard.getTab("Intake")
      .getLayout("Intake", BuiltInLayouts.kList)
      .withSize(2, 5)
      .withPosition(3, 0); 
    m_intakeMotorOnEntry = intakeLayout.add("Motor On", isIntakeMotorOn()).getEntry();
    m_intakeMotorEntry = intakeLayout.add("Motor Speed", m_intakeMotor.getMotorOutputPercent()).getEntry(); 
    m_intakeHasBallEntry = intakeLayout.add("Has Ball", intakeHasBall()).getEntry();
  
    // Feeder
    ShuffleboardLayout feederLayout = Shuffleboard.getTab("Intake")
      .getLayout("Feeder", BuiltInLayouts.kList)
      .withSize(2, 5)
      .withPosition(5, 0); 
    m_feederBrakeEnabledEntry = feederLayout.add("Switch Enabled", isFeederBrakeEnabled()).getEntry();
    m_feederMotorOnEntry = feederLayout.add("Motor On", isFeederMotorOn()).getEntry();
    m_feederMotorEntry = feederLayout.add("Motor Speed", m_rightFeederMotor.getMotorOutputPercent()).getEntry();  
    m_feederHasBallEntry = feederLayout.add("Has Ball", feederHasBall()).getEntry(); 

    // Ramp
    ShuffleboardLayout rampLayout = Shuffleboard.getTab("Intake")
      .getLayout("Ramp", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(8, 0); 
    m_rampEntry = rampLayout.add("Ramp Open?", isRampOpen()).getEntry();  

    // Commands
    m_commandsLayout = Shuffleboard.getTab("Intake")
      .getLayout("Commands", BuiltInLayouts.kList)
      .withSize(3, 6)
      .withProperties(Map.of("Label position", "HIDDEN")) // hide labels for commands
      .withPosition(0, 0);  
  }

  public void startMotors(){
    startFeederMotor(IntakeConstants.kFeederSpeed);
    startIntakeMotor(m_intakeMotorSpeed);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {

    // if (isIntakeSensorActivated() && feederHasBall()) {
    //   stopIntakeMotor();
    // } else {
    //   // Check if we commanded a stop from Operator Input
    //   if (intakeMotorStopRequired() == false) {      
    //     startIntakeMotor(m_intakeMotorSpeed);
    //   }     
    // }

    //if feeder has a ball, set intake brake enabled, otherwise set disabled
    if(feederHasBall()){
      setIntakeBrakeEnabled();
    } else {
      setIntakeBrakeDisabled();
    }

    //if either the feeder or intake is empty..
    // if(!(isIntakeSensorActivated() && feederHasBall())){
    //   // Check if we commanded a stop from Operator Input
    //   if (intakeMotorStopRequired() == false) {      
    //     startIntakeMotor(m_intakeMotorSpeed);
    //   }
    // }

    // if(m_drivetrain.getMotorOutputPercent() > -0.2){
    //     m_intakeMotorSpeed = 0;
    // } else {
    //     m_intakeMotorSpeed = IntakeConstants.kIntakeSpeed;
    // }
    

    // Intake motor will run at normal speed unless a low-speed timer has just been set
    // if (m_motorLowSpeedTimer.hasElapsed(2)) {
    //   m_intakeMotorSpeed = IntakeConstants.kIntakeSpeed;
    // }


    publishTelemetry();
    m_intakeMotorSpeed = m_intakeMotorSpeedEntry.getNumber(IntakeConstants.kIntakeSpeed).doubleValue(); 
  }

  public void publishTelemetry() {
    // Shuffleboard output
    m_intakeMotorOnEntry.setBoolean(isIntakeMotorOn());
    m_intakeMotorEntry.setNumber(m_intakeMotor.getMotorOutputPercent());
    m_feederMotorOnEntry.setBoolean(isFeederMotorOn());
    m_feederMotorEntry.setNumber(m_rightFeederMotor.getMotorOutputPercent());

    m_intakeHasBallEntry.setBoolean(intakeHasBall());
    m_feederHasBallEntry.setBoolean(feederHasBall());

    m_feederBrakeEnabledEntry.setBoolean(isFeederBrakeEnabled());

    m_rampEntry.setBoolean(isRampOpen());
  }

  public void ejectBall() {
    if (feederHasBall() && m_ejectInProgress == false) {
      m_ejectInProgress = true;
      openRamp();
      // Set the timer to wait until the ramp is open
      m_ejectDuration = 0.2;
      m_ejectTimer.reset();
      m_ejectTimer.start();
    }
  
    if (m_ejectTimer.hasElapsed(m_ejectDuration) && m_ejectInProgress) {
      if (isRampOpen() && isFeederBrakeEnabled()) {
        setFeederBrakeDisabled();
        startFeederMotor(IntakeConstants.kFeederSpeed);
        // Reset the timer to wait for the ball to leave
        m_ejectDuration = 1.0;
        m_ejectTimer.reset();
        m_ejectTimer.start();
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

  // --------- Intake Motor ------------------------------

  public void stopIntakeMotor(){
    System.out.println("intake motor stopping");
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  // /**
  //  * Delays the stopping of the intake motor for 0.1 seconds
  //  * This must be run in a periodic loop.
  //  */
  // public void stopIntakeMotorDelayed(){
  //   System.out.println("in stop intake motor delayed");

  //   if (m_startIntakeTimer) {
  //     m_intakeTimer.reset();
  //     m_intakeTimer.start();  // Tested in else below
  //     m_startIntakeTimer = false;
  //   } 
  //   else if (m_intakeTimer.hasElapsed(1)) {
  //       stopIntakeMotor();
  //       m_startIntakeTimer = true;
        
  //       // Now lower the intake speed for a short period in case the 
  //       // intake sensor bounces.
  //       m_intakeMotorSpeed = IntakeConstants.kIntakeLowSpeed;
  //       m_motorLowSpeedTimer.reset();
  //       m_motorLowSpeedTimer.start(); // Tested in periodic()
  //     }  
  //   }    
  

  /**
   * 
   * @param output the output of the motor, between -1 and 1
   */
  public void startIntakeMotor(double output){
    m_intakeMotor.set(ControlMode.PercentOutput, output);
  }

  public void startIntakeMotor(){
    m_intakeMotor.set(ControlMode.PercentOutput, m_intakeMotorSpeed);
  }

  public void setIntakeMotorStop(boolean state) {
    m_intakeMotorStop = state;
  }

  public boolean intakeMotorStopRequired() {
    return m_intakeMotorStop;
  }

  public void setLowIntakePower(){
    m_intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.kIntakeLowSpeed);
  }

  public void setIntakeBrakeEnabled(){
    m_intakeMotor.overrideLimitSwitchesEnable(true);
  }

  public void setIntakeBrakeDisabled(){
    m_intakeMotor.overrideLimitSwitchesEnable(false);
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
    if (isRampClosed()) {
      return true;
    } 
    return false;
  }

  // --------- Intake ------------------------------  


  public boolean intakeHasBall(){
    return isIntakeSensorActivated();
  }

  public boolean intakeCleared() {
    return !intakeHasBall();
  }


  public boolean isIntakeMotorOn(){
    //not percent, range between -1 and 1
    return (Math.abs(m_intakeMotor.getMotorOutputPercent()) > .1);
  }

  // public boolean isLeftIntakeSensorTripped(){
  //   return !m_leftIntakeSensor.get();
  // }

  // public boolean isRightIntakeSensorTripped(){
  //   return !m_rightIntakeSensor.get();
  // }

  public boolean isIntakeSensorActivated(){
    return m_intakeMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  // public boolean isIntakeSensorTripped() {
  //   return isLeftIntakeSensorTripped() || isRightIntakeSensorTripped();
  // }


  // --------- Feeder ------------------------------

  public boolean isFeederSwitchActivated() {
    // Simulate this return if not running on the real robot
    if (RobotBase.isReal()) {
      return !(m_rightFeederMotor.getSensorCollection().isFwdLimitSwitchClosed());
    }
    return m_intakeSim.isFeederSwitchClosed();
  }

  // public boolean isFeederBrakeDeactivated(){
  //   return !(isFeederBrakeActivated());
  // }
   
  public boolean feederHasBall(){
    return isFeederSwitchActivated();  
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

  
  public boolean isRampOpen(){
    // Simulate this return if not running on the real robot
    if (RobotBase.isReal()) {
      return (m_rampSolenoid.get());
    }  
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
    if (intakeHasBall()) {
      m_intakeMotorSim.setBusVoltage(0);
    } else {
      m_intakeMotorSim.setBusVoltage(RobotController.getInputVoltage());
    }

    if (feederHasBall()) {
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
    m_intakeMotorSim.setQuadratureVelocity((int)m_intakeSim.getOutput(0));
    m_rightFeederMotorSim.setQuadratureVelocity((int)m_intakeSim.getOutput(0));
  }  
  
  public void triggerCloseIntakeSwitchSim() {
    m_intakeSim.triggerCloseIntakeSwitchSim();
  }

}

