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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants;


public class Flywheel extends SubsystemBase {

  private final WPI_TalonFX m_flywheelTalon = new WPI_TalonFX(Constants.CANBusIDs.kFlywheelTalonLower);
  private final WPI_TalonFX m_flywheelTalonUpper = new WPI_TalonFX(Constants.CANBusIDs.kFlywheelTalonUpper);
  double m_velocity;
  double m_adjustableVelocity;
  double m_velocityChange;
  double m_upperVelocityChange;
  boolean m_isFlywheelSpunUp = false;

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
    m_velocityChange = .8;
    m_upperVelocityChange = 1;
    DistanceMap.getInstance().loadMaps();
    m_adjustableVelocity = FlywheelConstants.kIdealVelocity;
    m_shootTimer.reset();
    m_shootTimer.start();
  }

  public void configMotors(){

    for(TalonFX fx : new TalonFX[]{m_flywheelTalonUpper, m_flywheelTalon}){
    //Reset settings for safety
    fx.configFactoryDefault();

    //Sets voltage compensation to 12, used for percent output
    fx.configVoltageCompSaturation(10);
    fx.enableVoltageCompensation(true);

    //Setting just in case
    fx.configNominalOutputForward(0);
    fx.configNominalOutputReverse(0);
    fx.configPeakOutputForward(1);
    fx.configPeakOutputReverse(-1);

    fx.configOpenloopRamp(0.2);
  

    //Setting deadband(area required to start moving the motor) to 1%
    fx.configNeutralDeadband(0.01);

    //Set to brake mode, will brake the motor when no power is sent
    fx.setNeutralMode(NeutralMode.Coast);

    /** 
     * Setting input side current limit (amps)
     * 45 continious, 80 peak, 30 millieseconds allowed at peak
     * 40 amp breaker can support above 40 amps for a little bit
     * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
     */
    fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

    //Either using the integrated Falcon sensor or an external one, will change if needed
    fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
    }
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

    m_flywheelTalonUpper.config_kP(0, .02, 0);
    m_flywheelTalonUpper.config_kI(0, 0, 0);
    m_flywheelTalonUpper.config_kD(0, 0, 0);
    m_flywheelTalonUpper.config_kF(0, .2, 0);

    m_flywheelTalonUpper.config_kP(1, .02, 0);
    m_flywheelTalonUpper.config_kI(1, 0, 0);
    m_flywheelTalonUpper.config_kD(1, 0, 0);
    m_flywheelTalonUpper.config_kF(1, .2, 0);

    m_flywheelTalonUpper.config_kP(2, .02, 0);
    m_flywheelTalonUpper.config_kI(2, 0, 0);
    m_flywheelTalonUpper.config_kD(2, 0, 0);
    m_flywheelTalonUpper.config_kF(2, .2, 0);

    m_flywheelTalonUpper.config_kP(3, .02, 0);
    m_flywheelTalonUpper.config_kI(3, 0, 0);
    m_flywheelTalonUpper.config_kD(3, 0, 0);
    m_flywheelTalonUpper.config_kF(3, .2, 0);

  }

  // public void setupShuffleboard() {
  //   ShuffleboardTab m_flywheelTab = Shuffleboard.getTab("Flywheel"); 

  //   m_flywheelLayout = Shuffleboard.getTab("Flywheel")
  //           .getLayout("Flywheels", BuiltInLayouts.kList)
  //           .withSize(2, 2)
  //           .withPosition(0, 0);
  //         m_flywheelSpeedEntry = m_flywheelLayout.add("Speed in Ticks", getVelocity()).getEntry();
  //         m_flywheelPercentEntry = m_flywheelLayout.add
  //           ("Percent Output", m_flywheelMotorSim.getMotorOutputLeadVoltage()).getEntry();       
  //   m_flywheelTicksEntry = m_flywheelTab.add("Ticks Per 100 MS", getVelocity())
  //           .withSize(3,3)
  //           .withWidget(BuiltInWidgets.kGraph)
  //           .withPosition(5, 0)
  //           .getEntry();

  //   m_commandsLayout = Shuffleboard.getTab("Flywheel")
  //           .getLayout("Commands", BuiltInLayouts.kList)
  //           .withSize(3, 3)
  //           .withProperties(Map.of("Label position", "HIDDEN")) // hide labels for commands
  //           .withPosition(2, 0);
  // }

  public ShuffleboardLayout getCommandsLayout() {
    return m_commandsLayout;
  }


  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  @Override
  public void periodic() {   
    publishTelemetry();
    System.out.println(getVelocity());
    if(m_shootTimer.hasElapsed(2)){
    m_shootTimer.reset();
    m_shootTimer.start();
    }
  }

  public void publishTelemetry() {

    SmartDashboard.putNumber("Flywheel Speed (Front-Side)", m_flywheelTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Flywheel Speed (Back-Side)", m_flywheelTalonUpper.getSelectedSensorVelocity());



    // m_flywheelPercentEntry.setNumber(m_flywheelTalon.getMotorOutputPercent());
    // m_flywheelSpeedEntry.setNumber(m_flywheelTalon.getSelectedSensorVelocity());
    // m_flywheelTicksEntry.setNumber(m_flywheelTalon.getSelectedSensorVelocity());
  }

  public void resetEncoders(){
    m_flywheelTalon.setSelectedSensorPosition(0);
    m_flywheelTalonUpper.setSelectedSensorPosition(0);
  }

  /**
   * Set the PID values based on the required flywheel speed.
   * 
   * @param flywheelTicksPer100ms flywheel speed in tick per/100ms
   */
  public void setPidGains(int flywheelTicksPer100ms) {
    if(flywheelTicksPer100ms > 16000){
      m_flywheelTalon.selectProfileSlot(3, 0);
      m_flywheelTalonUpper.selectProfileSlot(3, 0);
    } else if (flywheelTicksPer100ms > 13000) {
      m_flywheelTalon.selectProfileSlot(1, 0);
      m_flywheelTalonUpper.selectProfileSlot(1, 0);
    } else if (flywheelTicksPer100ms > 8500) {
      m_flywheelTalon.selectProfileSlot(0, 0);
      m_flywheelTalonUpper.selectProfileSlot(0, 0);
    } else {
      m_flywheelTalon.selectProfileSlot(2, 0);
      m_flywheelTalonUpper.selectProfileSlot(2, 0);
    }
  }

   /* sets velocity to whatever is stored as the velocity variable, multiplied by the change in velocity
   */
  public void setVelocity(){

    //sets as ticks per 100 ms
    if(m_adjustableVelocity * m_velocityChange >= 20000){
      m_flywheelTalon.set(ControlMode.Velocity, 20000);
      m_flywheelTalonUpper.set(ControlMode.Velocity, 20000);
    } else {
      m_flywheelTalon.set(ControlMode.Velocity, m_adjustableVelocity * m_velocityChange);
      m_flywheelTalonUpper.set(ControlMode.Velocity, m_adjustableVelocity * m_upperVelocityChange * m_velocityChange);
    }
  }

  public void setPower(double power) {
    System.out.println("Power " + power);
    m_flywheelTalon.set(ControlMode.PercentOutput, power);
    m_flywheelTalonUpper.set(ControlMode.PercentOutput, power);
  }

  
  public void turnFlywheelOff() {
    m_flywheelTalon.set(ControlMode.Velocity, 0);
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
    System.out.println(m_velocityChange);
  }

  public void decreaseFlywheelChange(){
    m_velocityChange -= .02;
    System.out.println(m_velocityChange);
  }

  public void increaseUpperFlywheelChange(){
    m_upperVelocityChange += .05;
    System.out.println(m_upperVelocityChange);
  }

  public void decreaseUpperFlywheelChange(){
    m_upperVelocityChange -= .05;
    System.out.println(m_upperVelocityChange);
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
    return(m_flywheelTalon.getMotorOutputPercent() > .05);
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
