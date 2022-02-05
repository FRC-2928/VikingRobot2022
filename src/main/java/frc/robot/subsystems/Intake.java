// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightData;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonSRX m_intakeMotor  = new TalonSRX(Constants.RobotMap.kIntakeSparkMax);
  private NetworkTableEntry m_targetHOEntry;


  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Intake() {
    configMotors();
    setIntakePIDF();
    resetEncoders();
    
  }

  public void configMotors(){
      //Reset settings for safety
     m_intakeMotor.configFactoryDefault();

      //Sets voltage compensation to 12, used for percent output
     m_intakeMotor.configVoltageCompSaturation(10);
     m_intakeMotor.enableVoltageCompensation(true);

      //Setting just in case
     m_intakeMotor.configNominalOutputForward(0);
     m_intakeMotor.configNominalOutputReverse(0);
     m_intakeMotor.configPeakOutputForward(1);
     m_intakeMotor.configPeakOutputReverse(-1);

     m_intakeMotor.configOpenloopRamp(0.1);

      //Setting deadband(area required to start moving the motor) to 1%
     m_intakeMotor.configNeutralDeadband(0.01);

      //Set to brake mode, will brake the motor when no power is sent
     m_intakeMotor.setNeutralMode(NeutralMode.Coast);

      /** 
       * Setting input side current limit (amps)
       * 45 continious, 80 peak, 30 millieseconds allowed at peak
       * 40 amp breaker can support above 40 amps for a little bit
       * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
       */
     m_intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

      //Either using the integrated Falcon sensor or an external one, will change if needed
     m_intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
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

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

  
  
}

