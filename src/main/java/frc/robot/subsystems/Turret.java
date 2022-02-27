// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightData;
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
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final Drivetrain m_drivetrain;
  private final Limelight m_turretLimelight = new Limelight();
  private LimelightData m_turretLimelightData = m_turretLimelight.getLimelightData();
  private final TalonSRX m_turretMotor  = new TalonSRX(Constants.CANBusIDs.kTurretTalonSRX);
  private NetworkTableEntry m_targetHOEntry;
  private NetworkTableEntry m_turretTicksEntry, m_turretPowerEntry;
  private NetworkTableEntry m_turretOffsetEntry, m_targetOffsetEntry;
  private NetworkTableEntry m_targetLockedEntry, m_targetFoundEntry, m_estimatedTargetRotationEntry;
  LinearFilter m_filter = LinearFilter.movingAverage(5);
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
  // private double m_lastHeading = 0.0;
  // private double m_targetRobotOffset = 0.0;
  private Rotation2d m_lastHeading = new Rotation2d();
  private Rotation2d m_targetRobotOffset = new Rotation2d();
  private Rotation2d m_estimatedTargetRotation = new Rotation2d();

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Turret(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    configMotors();
    setTurretPIDF();
    resetEncoders();
    setupShuffleboard();
    m_turretPose = new Pose2d(0,0,getTargetToRobotOffset());
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

    //  m_turretMotor.configForwardLimitSwitchSource(type, normalOpenOrClose);
    //  m_turretMotor.configReverseLimitSwitchSource(type, normalOpenOrClose);
    m_turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
                                                 LimitSwitchNormal.NormallyClosed, 0);

    m_turretMotor.setInverted(true);
    m_turretMotor.setSensorPhase(true);

    m_turretMotor.configForwardSoftLimitThreshold(2750);
    m_turretMotor.configReverseSoftLimitThreshold(-2750);
    m_turretMotor.configForwardSoftLimitEnable(true, 0);
    m_turretMotor.configReverseSoftLimitEnable(true, 0);
    m_turretMotor.overrideSoftLimitsEnable(true);
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
      .withSize(3,3)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(1, 0)
      .getEntry(); 
    m_turretPowerEntry = m_turretTab.add("Motor Power", m_turretMotor.getMotorOutputPercent())
      .withSize(3,3)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(5, 0)
      .getEntry();
    m_turretTicksEntry = m_turretTab.add("Turret Ticks", m_turretMotor.getSelectedSensorPosition())
      .withSize(2,1)
      .withPosition(1, 5)
      .getEntry();  
    m_turretOffsetEntry = m_turretTab.add("Turret Offset", getTurretToRobotOffset().getDegrees())
      .withSize(2,1)
      .withPosition(3, 5)
      .getEntry(); 
    m_targetOffsetEntry = m_turretTab.add("Target Offset", getTargetToRobotOffset().getDegrees())
      .withSize(2,1)
      .withPosition(5, 5)
      .getEntry(); 
    m_estimatedTargetRotationEntry = m_turretTab.add("Estimated Offset", getEstimatedTargetRotation().getDegrees())
      .withSize(2,1)
      .withPosition(7, 5)
      .getEntry();   
      
    ShuffleboardLayout targetLayout = Shuffleboard.getTab("Turret")
      .getLayout("Target", BuiltInLayouts.kList)
      .withSize(2, 5)
      .withPosition(9, 0);  
    m_targetLockedEntry = targetLayout.add("Locked", getTargetLocked()).getEntry();
    m_targetFoundEntry = targetLayout.add("Found", getTargetFound()).getEntry();

    // SmartDashboard.putData("Target Pose", m_field2d);  
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    
    // get the offset from the target heading to the robot heading,
    // but only if we have found the target.
    if(getTargetFound()) {
      // m_lastHeading = m_drivetrain.getRotation().getDegrees();
      // m_targetRobotOffset = getTargetToRobotOffset().getDegrees();
      m_lastHeading = m_drivetrain.getRotation();
      m_targetRobotOffset = getTargetToRobotOffset();
    }

    // calculate the estimated target rotation.                           
    // m_estimatedTargetRotation = Rotation2d.fromDegrees((m_targetRobotOffset + m_lastHeading) - m_drivetrain.getRotation().getDegrees());
    Rotation2d currentHeading = m_drivetrain.getRotation();
    m_estimatedTargetRotation = m_targetRobotOffset
                                  .plus(m_lastHeading)
                                  .minus(currentHeading);

    publishTelemetry();
  }

  public void publishTelemetry() {
    m_turretTicksEntry.setNumber(m_turretMotor.getSelectedSensorPosition());
    m_targetHOEntry.setNumber(targetHorizontalOffset());
    m_turretPowerEntry.setNumber(m_turretMotor.getMotorOutputPercent());
    m_turretOffsetEntry.setNumber(getTurretToRobotOffset().getDegrees());
    m_targetOffsetEntry.setNumber(getTargetToRobotOffset().getDegrees());
    m_estimatedTargetRotationEntry.setNumber(getEstimatedTargetRotation().getDegrees());

    m_targetLockedEntry.setBoolean(getTargetLocked());
    m_targetFoundEntry.setBoolean(getTargetFound());
    
    // Using this to display the angle of the target from the robot heading
    // m_field2d.setRobotPose(getTargetPose());

    // SmartDashboard.putBoolean("Target found", m_turretLimelight.isTargetFound());
    SmartDashboard.putNumber("Target X", m_turretLimelight.getHorizontalOffset());
    SmartDashboard.putNumber("Target Y", m_turretLimelight.getVerticalOffset());
    SmartDashboard.putNumber("Target Skew", m_turretLimelight.getSkew());
    SmartDashboard.putNumber("Target Area", m_turretLimelight.getArea());
    // SmartDashboard.putNumber("Turret Degrees", encoderTicksToDegrees( m_turretMotor.getSelectedSensorPosition()));
  }

  public void resetEncoders(){
    m_turretMotor.setSelectedSensorPosition(0);
  }

  public void rotateTurret(DoubleSupplier rotate){
    setPower(rotate.getAsDouble());
  }

  /**
   * Moves the turret clockwise or anti-clockwise the specified angle
   * @param angleDegrees the position to move the turret to. 
   */
  public void setTurretDegrees(double angleDegrees) {
    double encoderTicks = getDegreesToEncoderTicks(angleDegrees);
    m_turretMotor.set(ControlMode.Position, encoderTicks);
 
    SmartDashboard.putNumber("turret heading offset", angleDegrees);      
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
   * resets the encoder to the given tick value
   * @param ticks the number of ticks the encoder should be set to
   */
  public void setSensorTicks(double ticks){
    m_turretMotor.setSelectedSensorPosition(ticks);
  }

  public void setTurretBrakeEnabled(){
    m_turretMotor.overrideLimitSwitchesEnable(false);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public Pose2d getTargetPose() {
    m_turretPose = new Pose2d(0,0, getTargetToRobotOffset());
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
   * Gets the angle of the target relative to the turret
   * @return offset angle between target and the turret
   */
  public double targetHorizontalOffset() {
    double offset = m_turretLimelight.getHorizontalOffset();
    // pass it though an averaging filter
    return m_filter.calculate(offset);
  }

  /**
   * Gets the angle of the target relative to the turret
   * @return offset angle between target and the turret in Radians
   */
  public Rotation2d getTargetToTurretOffset() {
    double offset = targetHorizontalOffset();
    return (Rotation2d.fromDegrees(offset));
  }

  /**
   * Gets the angle of the turret relative to the robot chassis
   * @return offset angle between turret and robot chassis in Radians
   */
  public Rotation2d getTurretToRobotOffset() {
    double offset = m_drivetrain.getRotation().getDegrees() - getTurretDegrees();
    return (Rotation2d.fromDegrees(offset));
  }

  /**
   * Gets the angle of the target relative to the robot chassis
   * @return offset angle between target and robot chassis in Radians
   */
  public Rotation2d getTargetToRobotOffset() {
    double offset = getTurretToRobotOffset().getDegrees() + getTargetToTurretOffset().getDegrees();
    return (Rotation2d.fromDegrees(offset));
  }

  public boolean getTargetFound() {
    return m_turretLimelight.isTargetFound();
  }

  public boolean getTargetLocked() {
    if (getTargetFound()) {
      return Math.abs(targetHorizontalOffset()) < 2;
    }
    return false;
  }


  public boolean isLimitSwitchClosed(){
    return m_turretMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  // -----------------------------------------------------------
    // Simulation
    // -----------------------------------------------------------
    public void simulationPeriodic() {
      /* Pass the robot battery voltage to the simulated Talon FXs */
      m_turretMotorSim.setBusVoltage(RobotController.getInputVoltage());
      // System.out.println("Input Voltage " + m_turretMotorSim.getMotorOutputLeadVoltage());

      m_turretSim.setInput(m_turretMotorSim.getMotorOutputLeadVoltage());  
      
      /*
       * Advance the model by 20 ms. Note that if you are running this
       * subsystem in a separate thread or have changed the nominal
       * timestep of TimedRobot, this value needs to match it.
       */
      m_turretSim.update(0.02);
  
      // /*
      //  * Update all of the sensors.
      //  *
      //  * Since WPILib's simulation class is assuming +V is forward,
      //  * but -V is forward for the right motor, we need to negate the
      //  * position reported by the simulation class. Basically, we
      //  * negated the input, so we need to negate the output.
      //  */
      m_turretMotorSim.setQuadratureRawPosition((int)m_turretSim.getOutput(0)*10);
      
    }

}
