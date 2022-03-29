// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants;
import frc.robot.simulation.FlywheelSim;


public class Flywheel extends SubsystemBase {

  private final WPI_TalonFX m_flywheelTalon = new WPI_TalonFX(Constants.CANBusIDs.kFlywheelTalonFX);
  double m_velocity;
  double m_adjustableVelocity;
  double m_velocityChange;
  boolean m_isFlywheelSpunUp = false;

  //simulation
  TalonFXSimCollection m_flywheelMotorSim = m_flywheelTalon.getSimCollection();
  
  FlywheelSim m_flywheelSim = new FlywheelSim(FlywheelConstants.kFlywheelLinearSystem);

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
    //setupShuffleboard();
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

  public void setupShuffleboard() {
    ShuffleboardTab m_flywheelTab = Shuffleboard.getTab("Flywheel"); 

    m_flywheelLayout = Shuffleboard.getTab("Flywheel")
            .getLayout("Flywheels", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0);
          m_flywheelSpeedEntry = m_flywheelLayout.add("Speed in Ticks", getVelocity()).getEntry();
          m_flywheelPercentEntry = m_flywheelLayout.add
            ("Percent Output", m_flywheelMotorSim.getMotorOutputLeadVoltage()).getEntry();       
    m_flywheelTicksEntry = m_flywheelTab.add("Ticks Per 100 MS", getVelocity())
            .withSize(3,3)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(5, 0)
            .getEntry();

    m_commandsLayout = Shuffleboard.getTab("Flywheel")
            .getLayout("Commands", BuiltInLayouts.kList)
            .withSize(3, 3)
            .withProperties(Map.of("Label position", "HIDDEN")) // hide labels for commands
            .withPosition(2, 0);
  }

  public ShuffleboardLayout getCommandsLayout() {
    return m_commandsLayout;
  }


  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  @Override
  public void periodic() {   
    //publishTelemetry();
    if(m_shootTimer.hasElapsed(2)){
    System.out.println(getVelocity());
    System.out.println(m_adjustableVelocity);
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

  /**
   * 
   * @param velocity change in ticks per 100 ms
   */
  public void setVelocity(double velocity){

    //turn change in ticks per sec to change in ticks per 100 ms
    m_flywheelTalon.set(ControlMode.Velocity, velocity);
  }

  public void setVelocity(){

    //sets as ticks per 100 ms
    m_flywheelTalon.set(ControlMode.Velocity, m_adjustableVelocity * m_velocityChange);
  }

  public void setPower(double power) {
    System.out.println("Power " + power);
    m_flywheelTalon.set(ControlMode.PercentOutput, power);
  }

  public void incrementVelocity(double increment){
    m_velocity += increment;
    setPower(m_velocity);    
  }

  public void decrementVelocity(double decrement){
    if(m_velocity <= 0){
      setPower(0);
    } else{
      m_velocity -= decrement;
      setPower(m_velocity);
    }
  }

  public int calculateFlywheelTicksPer100ms(int distance) {
    int ticks = DistanceMap.getInstance().getFlywheelTicksPer100ms(distance);
    return ticks;
  }

  public void setAdjustableVelocity(double velocity){
    m_adjustableVelocity = velocity;
  }

  public void stopFlywheel(){
    setAdjustableVelocity(0);
    setPower(0);
  }

  public boolean isFlyWheelUpToSpeed(){

    if(Math.abs(m_adjustableVelocity - getVelocity()) <= 200.0){
      return true;

    }else{

      return false;
    }
  }




  // -----------------------------------------------------------
  // System State
  // --------------------P---------------------------------------
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
   * @return the percent output of the motor (-1 to 1)
   */
  public double getPower(){
    return(m_flywheelTalon.getMotorOutputPercent());
  }

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



  // -----------------------------------------------------------
  // Simulation
  // -----------------------------------------------------------

  public void simulationPeriodic(){

    //set voltage from voltage from robot controller
    m_flywheelMotorSim.setBusVoltage(RobotController.getInputVoltage());

    //set input to simulator as output voltage from sim motor
    m_flywheelSim.setInput(m_flywheelMotorSim.getMotorOutputLeadVoltage());

    //update time by 20 ms
    m_flywheelSim.update(0.02);

    //set the encoder values from the sim motor's output
    m_flywheelMotorSim.setIntegratedSensorVelocity((int)m_flywheelSim.getOutput(0));
  
  }
}
