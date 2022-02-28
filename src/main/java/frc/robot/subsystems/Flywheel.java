// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
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

  //simulation
  TalonFXSimCollection m_flywheelMotorSim = m_flywheelTalon.getSimCollection();
  
  FlywheelSim m_flywheelSim = new FlywheelSim(FlywheelConstants.kFlywheelLinearSystem);

  //for srx use sim motor
  FlywheelSim m_flywheelSimMotor = new FlywheelSim(DCMotor.getFalcon500(1), 
                                                    FlywheelConstants.kFlywheelRadius, 
                                                    FlywheelConstants.kFlywheelMass, 
                                                    FlywheelConstants.kGearRatio);

  ShuffleboardLayout m_flywheelLayout;
  NetworkTableEntry m_flywheelSpeedEntry;
  NetworkTableEntry m_flywheelVoltageEntry;
  private ShuffleboardLayout m_commandsLayout;


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

    m_flywheelTalon.setInverted(true);
  }

  public void setFlywheelPIDF() {
    m_flywheelTalon.config_kP(0, FlywheelConstants.kGainsVelocity.kP, 0);
    m_flywheelTalon.config_kI(0, FlywheelConstants.kGainsVelocity.kI, 0);
    m_flywheelTalon.config_kD(0, FlywheelConstants.kGainsVelocity.kD, 0);
    m_flywheelTalon.config_kF(0, FlywheelConstants.kGainsVelocity.kF, 0);
  }

  public void setupShuffleboard() {
    ShuffleboardTab m_flywheelTab = Shuffleboard.getTab("Flywheel"); 

    m_flywheelLayout = Shuffleboard.getTab("Flywheel")
            .getLayout("Flywheels", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0);
          m_flywheelSpeedEntry = m_flywheelLayout.add("Flywheel Motor Speed", getVelocity()).getEntry();
          m_flywheelVoltageEntry = m_flywheelLayout.add
            ("Sim Motor Voltage", m_flywheelMotorSim.getMotorOutputLeadVoltage()).getEntry();
          
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
    publishTelemetry();
  }

  public void publishTelemetry() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Flywheel Motor Percent", m_flywheelTalon.getMotorOutputPercent());
    // SmartDashboard.putNumber("Flywheel Motor Voltage", m_flywheelTalon.getMotorOutputVoltage());

    m_flywheelVoltageEntry.setNumber(m_flywheelTalon.getMotorOutputVoltage());
    m_flywheelSpeedEntry.setNumber(m_flywheelTalon.getSelectedSensorVelocity());
  }

  public void resetEncoders(){
    m_flywheelTalon.setSelectedSensorPosition(0);
  }

  /**
   * 
   * @param velocity change in ticks per sec
   */
  public void setVelocity(double velocity){

    //turn change in ticks per sec to change in ticks per 100 ms
    m_flywheelTalon.set(ControlMode.Velocity, velocity/10);

    // TODO may need to add FeedForward using SimpleMotorFeedforward
    // m_flywheelTalon.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, velocity);
  }

  public void setPower(double power) {
    System.out.println("Power " + power);
    m_flywheelTalon.set(ControlMode.PercentOutput, power);
  }

  public void incrementVelocity(double increment){
    if(getVelocity() < FlywheelConstants.kMaxVelocity){
      m_velocity += increment;
      setPower(m_velocity);
    } else {
      setPower(FlywheelConstants.kIdealVelocity);
    }
  }

  public void decrementVelocity(double decrement){
    if(m_velocity <= 0){
      setPower(0);
    } else{
      m_velocity -= decrement;
      setPower(m_velocity);
    }
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
   * @return the percent output of the motor (-1 to 1)
   */
  public double getPower(){
    return(m_flywheelTalon.getMotorOutputPercent());
  }

  /**
   * 
   * @return velocity in ticks per sec
   */
  public double getVelocity(){
    return (m_flywheelTalon.getSelectedSensorVelocity() * 10);

    
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
    m_flywheelMotorSim.setIntegratedSensorRawPosition((int)m_flywheelSim.getOutput(0));

    // m_flywheelSpeedEntry.setNumber(m_flywheelTalon.getMotorOutputPercent());
    // m_flywheelVoltageEntry.setNumber(m_flywheelTalon.getSelectedSensorVelocity());
  
  }


  

}
