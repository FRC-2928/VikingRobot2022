// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightData;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final Limelight m_turretLimelight = new Limelight();
  private LimelightData m_turretLimelightData = m_turretLimelight.getLimelightData();
  private final TalonSRX m_turretMotor  = new TalonSRX(Constants.RobotMap.kTurretTalonSRX);
  private NetworkTableEntry m_targetHOEntry;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Turret() {
    configMotors();
    setTurretPIDF();
    resetEncoders();
    setupShuffleboard();
  }

  public void configMotors(){
      //Reset settings for safety
     m_turretMotor.configFactoryDefault();

      //Sets voltage compensation to 12, used for percent output
     m_turretMotor.configVoltageCompSaturation(10);
     m_turretMotor.enableVoltageCompensation(true);

      //Setting just in case
     m_turretMotor.configNominalOutputForward(0);
     m_turretMotor.configNominalOutputReverse(0);
     m_turretMotor.configPeakOutputForward(1);
     m_turretMotor.configPeakOutputReverse(-1);

     m_turretMotor.configOpenloopRamp(0.1);

      //Setting deadband(area required to start moving the motor) to 1%
     m_turretMotor.configNeutralDeadband(0.01);

      //Set to brake mode, will brake the motor when no power is sent
     m_turretMotor.setNeutralMode(NeutralMode.Coast);

      /** 
       * Setting input side current limit (amps)
       * 45 continious, 80 peak, 30 millieseconds allowed at peak
       * 40 amp breaker can support above 40 amps for a little bit
       * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
       */
     m_turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

      //Either using the integrated Falcon sensor or an external one, will change if needed
     m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 

    //  m_turretMotor.configForwardLimitSwitchSource(type, normalOpenOrClose);
    //  m_turretMotor.configReverseLimitSwitchSource(type, normalOpenOrClose);
  }

  public void setTurretPIDF() {
    m_turretMotor.config_kP(0, TurretConstants.kGainsTurret.kP, 0);
    m_turretMotor.config_kI(0, TurretConstants.kGainsTurret.kI, 0);
    m_turretMotor.config_kD(0, TurretConstants.kGainsTurret.kD, 0);
    m_turretMotor.config_kF(0, TurretConstants.kGainsTurret.kF, 0);
  }

  public void setupShuffleboard() {
    ShuffleboardTab m_turretTab = Shuffleboard.getTab("Turret"); 
    m_targetHOEntry = m_turretTab.add("Target Horizontal Offset", targetHorizontalOffset())
            .withSize(2,1)
            .withPosition(3, 0)
            .getEntry(); 
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Target found", m_turretLimelight.isTargetFound());
    SmartDashboard.putNumber("Target X", m_turretLimelight.getHorizontalOffset());
    SmartDashboard.putNumber("Target Y", m_turretLimelight.getVerticalOffset());
    SmartDashboard.putNumber("Target Skew", m_turretLimelight.getSkew());
    SmartDashboard.putNumber("Target Area", m_turretLimelight.getArea());
  }

  public void resetEncoders(){
    m_turretMotor.setSelectedSensorPosition(0);
  }

  public void setTurretDegrees(double angleDegrees) {
    double encoderTicks = getDegreesToEncoderTicks(angleDegrees);
    m_turretMotor.set(ControlMode.MotionMagic, encoderTicks);

      SmartDashboard.putBoolean("target found", m_turretLimelightData.getTargetFound());
      SmartDashboard.putNumber("skew", m_turretLimelightData.getSkew());
      SmartDashboard.putNumber("horizontal offset", m_turretLimelightData.getHorizontalOffset());
      SmartDashboard.putNumber("vertical offset", m_turretLimelightData.getVerticalOffset());      
    
  }

  /**
   * sets the power of the turret motor
   * @param power the power value between -1 and 1
   */
  public void setPower(double power){
    m_turretMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * resets the encoder to the given tick value
   * @param ticks the number of ticks the encoder should be set to
   */
  public void setSensorTicks(double ticks){
    m_turretMotor.setSelectedSensorPosition(ticks);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public double encoderTicksToDegrees(double encoderTicks) {
    // Convert encoder ticks to turret rotations
    double turretRotations = encoderTicks / (TurretConstants.kEncoderCPR * TurretConstants.kTurretGearRatio);
    // Convert turret rotations to degrees
    return turretRotations * 360;
  }

  public double getDegreesToEncoderTicks(double degrees) {
    // Convert degrees to turret rotations
    double turretRotations = degrees / 360;
    // Convert turret rotations to encoder ticks
    return turretRotations * TurretConstants.kEncoderCPR * TurretConstants.kTurretGearRatio;
  }

  public double getTurretDegrees() {
    return encoderTicksToDegrees(m_turretMotor.getSelectedSensorPosition());
  }

  public double targetHorizontalOffset() {
    double offset = m_turretLimelight.getHorizontalOffset();
    SmartDashboard.putNumber("Horizontal Offset", offset);
    return offset;
  }

  public boolean isLimitSwitchClosed(){
    return m_turretMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }
}
