// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.simulation.TurretSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import org.photonvision.SimVisionSystem;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final Drivetrain m_drivetrain;
  private final Flywheel m_flywheel;
  private final Limelight m_turretLimelight = new Limelight();
  private LimelightData m_turretLimelightData = m_turretLimelight.getLimelightData();
  private final TalonSRX m_turretMotor  = new TalonSRX(Constants.CANBusIDs.kTurretTalonSRX);
  private ShuffleboardLayout m_commandsLayout;
  private NetworkTableEntry m_targetHOEntry;
  private NetworkTableEntry m_headingOffsetEntry, m_turretPowerEntry;
  private NetworkTableEntry m_turretOffsetEntry, m_targetOffsetEntry;
  private NetworkTableEntry m_targetLockedEntry, m_targetFoundEntry, m_estimatedTargetRotationEntry;
  LinearFilter m_filter = LinearFilter.movingAverage(5);
  MedianFilter m_verticalFilter = new MedianFilter(10);
  private final Field2d m_field2d = new Field2d();
  private Pose2d m_turretPose;

  // ------ Simulation classes to help us simulate our robot ----------------
  TalonSRXSimCollection m_turretMotorSim = m_turretMotor.getSimCollection();
  TurretSim m_turretSim = new TurretSim(TurretConstants.kTurretLinearSystem);

  // Simulated Vision System.
  // Configure these to match your PhotonVision Camera,
  // pipeline, and LED setup.
  double camDiagFOV = 170.0; // degrees - assume wide-angle camera
  double camPitch = Units.radiansToDegrees(0.5); // degrees
  double camHeightOffGround = 1.0; // meters
  double maxLEDRange = 20; // meters
  int camResolutionWidth = 640; // pixels
  int camResolutionHeight = 480; // pixels
  double minTargetArea = 10; // square pixels

  SimVisionSystem simVision =
          new SimVisionSystem(
                  "photonvision",
                  camDiagFOV,
                  camPitch,
                  new Transform2d(),
                  camHeightOffGround,
                  maxLEDRange,
                  camResolutionWidth,
                  camResolutionHeight,
                  minTargetArea);

  // instance variables for handling targeting.
  private Rotation2d m_lastHeading = new Rotation2d();
  private Rotation2d m_targetHeadingOffset = new Rotation2d();
  private Rotation2d m_estimatedTargetRotation = new Rotation2d();

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Turret(Drivetrain drivetrain, Flywheel flywheel) {
    m_drivetrain = drivetrain;
    m_flywheel = flywheel;
    configMotors();
    setTurretPIDF();
    resetEncoders();
    m_turretPose = new Pose2d(0,0,getTargetToHeadingOffset());
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

     m_turretMotor.configOpenloopRamp(0);

      //Setting deadband(area required to start moving the motor) to 1%
     m_turretMotor.configNeutralDeadband(0.01);

      //Set to brake mode, will brake the motor when no power is sent
     m_turretMotor.setNeutralMode(NeutralMode.Brake);

      /** 
       * Setting input side current limit (amps)
       * 45 continious, 80 peak, 30 millieseconds allowed at peak
       * 40 amp breaker can support above 40 amps for a little bit
       * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
       */
     m_turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

      //Either using the integrated Falcon sensor or an external one, will change if needed
     m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); 

    m_turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                                 LimitSwitchNormal.NormallyOpen, 0);
    m_turretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                                 LimitSwitchNormal.NormallyOpen, 0);                                             

    m_turretMotor.setInverted(true);
    m_turretMotor.setSensorPhase(true);

    m_turretMotor.configForwardSoftLimitThreshold(7300);
    m_turretMotor.configReverseSoftLimitThreshold(-7300);
    m_turretMotor.configForwardSoftLimitEnable(false, 0);
    m_turretMotor.configReverseSoftLimitEnable(false, 0);
    m_turretMotor.overrideSoftLimitsEnable(false);
  }

  public void setTurretPIDF() {
    m_turretMotor.config_kP(0, TurretConstants.kGainsTurret.kP, 0);
    m_turretMotor.config_kI(0, TurretConstants.kGainsTurret.kI, 0);
    m_turretMotor.config_kD(0, TurretConstants.kGainsTurret.kD, 0);
    m_turretMotor.config_kF(0, TurretConstants.kGainsTurret.kF, 0);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is Target in Range (High)",inRange());
    if(getTargetFound()) {
      m_lastHeading = m_drivetrain.getRotation();
      m_targetHeadingOffset = getTargetToHeadingOffset();
    }

    // calculate the estimated target rotation.                           
    Rotation2d currentHeading = m_drivetrain.getRotation();
    m_estimatedTargetRotation = m_targetHeadingOffset
                                  .plus(m_lastHeading)
                                  .minus(currentHeading);

    publishTelemetry();
  }

  public void publishTelemetry() {   
    m_targetHOEntry.setNumber(getTargetHorizontalOffset());
    m_turretPowerEntry.setNumber(m_turretMotor.getMotorOutputPercent());
    m_turretOffsetEntry.setNumber(getTurretToRobotOffset().getDegrees());   
    m_headingOffsetEntry.setNumber(getTurretToHeadingOffset().getDegrees());
    m_targetOffsetEntry.setNumber(getTargetToHeadingOffset().getDegrees());
    m_estimatedTargetRotationEntry.setNumber(getEstimatedTargetRotation().getDegrees());

    m_targetLockedEntry.setBoolean(getTargetLocked());
    m_targetFoundEntry.setBoolean(getTargetFound());

    
    SmartDashboard.putNumber("Target X", m_turretLimelight.getHorizontalOffset());
    SmartDashboard.putNumber("Target Y", m_turretLimelight.getVerticalOffset());
    SmartDashboard.putNumber("Target Skew", m_turretLimelight.getSkew());
    SmartDashboard.putNumber("Target Area", m_turretLimelight.getArea());
    
  }

  public ShuffleboardLayout getCommandsLayout() {
    return m_commandsLayout;
  }

  public void resetEncoders(){
    m_turretMotor.setSelectedSensorPosition(0);
  }

  public boolean inRange(){

    return ((getTargetVerticalOffset() > 20 && getTargetVerticalOffset() < 40));
  }
  /**
   * Moves the turret to the specified angle
   * @param angleDegrees the absolute position to move the turret to. 
   */
  public void setTurretDegrees(double angleDegrees) {
    double encoderTicks = getDegreesToEncoderTicks(angleDegrees);
    m_turretMotor.set(ControlMode.MotionMagic, encoderTicks);
  
    SmartDashboard.putNumber("Turret angleDegrees", angleDegrees);
    SmartDashboard.putNumber("Turret ticks", encoderTicks);      
  }

  public boolean motionProfileFinished() {
    return m_turretMotor.isMotionProfileFinished();
  }

  /**
   * sets the power of the turret motor
   * @param power the power value between -1 and 1
   */
  public void setPower(double power){
    SmartDashboard.putNumber("Turret Power", power);
    m_turretMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Move turret left/right using an axis trigger
   * 
   * @param extend Move it left
   * @param retract Move it right
   */
  public void rotateTurret(DoubleSupplier left, DoubleSupplier right){
    double leftPower = MathUtil.applyDeadband(left.getAsDouble(), 0.02);
    double rightPower = MathUtil.applyDeadband(right.getAsDouble(), 0.02);
    if (leftPower > 0) {
      setPower(leftPower);
    } else {
      setPower(-rightPower);
    }
  }

  /**
   * resets the encoder to the given tick value
   * @param ticks the number of ticks the encoder should be set to
   */
  public void setSensorTicks(double ticks){
    m_turretMotor.setSelectedSensorPosition(ticks);
  }

  public void setTurretBrakeEnabled(){
    m_turretMotor.overrideLimitSwitchesEnable(false);
  }

  public boolean angleInRange(double angle) {
    return (angle < 120 & angle > -120);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public Pose2d getTargetPose() {
    m_turretPose = new Pose2d(0,0, getTargetToHeadingOffset());
    return m_turretPose;
  }

  public Rotation2d getEstimatedTargetRotation() {
    return m_estimatedTargetRotation;
  }
  
  public double encoderTicksToDegrees(double encoderTicks) {
    return encoderTicks / TurretConstants.kTurretTicksPerDegree;
  }

  public double getDegreesToEncoderTicks(double degrees) {
    return degrees * TurretConstants.kTurretTicksPerDegree;
  }

  public double getTurretDegrees() {
    return encoderTicksToDegrees(m_turretMotor.getSelectedSensorPosition());
  }

  /**
   * Gets the angle of the turret relative to the robot chassis
   * @return Rotation2d offset angle between turret and robot chassis in Radians
   */
  public Rotation2d getTurretToRobotOffset() {
    return (Rotation2d.fromDegrees(getTurretDegrees()));
  }

  /**
   * Gets the angle of the target relative to the turret
   * @return offset angle between target and the turret
   */
  public double getTargetHorizontalOffset() {
    double offset = m_turretLimelight.getHorizontalOffset();
    return m_filter.calculate(offset);
  }

  public int getTargetVerticalOffset(){
    double offset = m_turretLimelight.getVerticalOffset();
    return (int)(m_verticalFilter.calculate(offset));
  }

  /**
   * Gets the angle of the target relative to the turret
   * @return Rotation2d offset angle between target and the turret in Radians
   */
  public Rotation2d getTargetToTurretOffset() {
    double offset = getTargetHorizontalOffset();
    return (Rotation2d.fromDegrees(offset));
  }

  /**
   * Gets the angle of the turret relative to the robot heading
   * @return offset angle between turret and robot chassis in Radians
   */
  public Rotation2d getTurretToHeadingOffset() {
    double offset = m_drivetrain.getRotation().getDegrees() - getTurretDegrees();
    return (Rotation2d.fromDegrees(offset));
  }

  /**
   * Gets the angle of the target relative to the robot heading
   * @return offset angle between target and robot chassis in Radians
   */
  public Rotation2d getTargetToHeadingOffset() {
    Rotation2d offset = getTurretToHeadingOffset().plus(getTargetToTurretOffset());
    return offset;
  }

  public boolean getTargetFound() {
    return m_turretLimelight.isTargetFound();
  }

  public boolean getTargetLocked() {
    if (getTargetFound()) {
      return Math.abs(getTargetHorizontalOffset()) < 2;
    }
    return false;
  }

  public boolean isLimitSwitchClosed(){
    return m_turretMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

}
