// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final WPI_TalonSRX m_climberMotor = new WPI_TalonSRX(Constants.CANBusIDs.kClimberMotor);
  private final WPI_TalonSRX m_climberFollower = new WPI_TalonSRX(Constants.CANBusIDs.kClimberMotor2);
  
  Solenoid m_climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kClimberSolenoid);

  ShuffleboardTab m_climberTab;
  private NetworkTableEntry m_climberPowerEntry;
  
  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  
  /** Creates a new Climber. */
  public Climber() {
    configMotors();
    setClimberPIDF();
    tiltBack();
  }


  public void configMotors(){

    for(WPI_TalonSRX srx : new WPI_TalonSRX[] {m_climberMotor, m_climberFollower}) {

      //Reset settings for safety
      srx.configFactoryDefault();

      //Sets voltage compensation to 12, used for percent output
      srx.configVoltageCompSaturation(12);
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
      // m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); 
    }

    m_climberMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                                  LimitSwitchNormal.NormallyOpen, 0);
    
    m_climberFollower.follow(m_climberMotor, FollowerType.PercentOutput);

  }

  public void setClimberPIDF() {
    m_climberMotor.config_kP(0, ClimberConstants.kGainsClimber.kP , 0);
    m_climberMotor.config_kI(0, ClimberConstants.kGainsClimber.kI, 0);
    m_climberMotor.config_kD(0, ClimberConstants.kGainsClimber.kD, 0);
    m_climberMotor.config_kF(0, ClimberConstants.kGainsClimber.kF, 0);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  
  @Override
  public void periodic() {}

  /**
   * 
   * @param power percent power of the motor, between -1 and 1
   */
  public void setPower(double power){
    m_climberMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Move climber up/down using an axis trigger
   * Positive value moves up, negative moves down
   * 
   * @param extendRetract value provided by double supplier (left axis' Y value)
   */
  public void moveClimber(DoubleSupplier extendRetract){
    double power = MathUtil.applyDeadband(extendRetract.getAsDouble(), 0.07);
    setPower(-power);
  }
  
  public void tiltForward(){
    m_climberSolenoid.set(false);
  }

  public void tiltBack(){
    m_climberSolenoid.set(true);
  }


  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  public boolean isClimberForward(){
    return m_climberSolenoid.get();
  }
}
