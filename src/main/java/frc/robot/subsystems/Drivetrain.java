package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
/**
   * DrivetrainSubsystem handles all subsystem level logic for the drivetrain.
   * Possibly also Ramsete idfk I haven't finished this class yet.
   */
import frc.robot.subsystems.Pigeon;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX m_leftLeader, m_rightLeader;
    private WPI_TalonFX m_leftFollower, m_rightFollower;

    private Pigeon m_pigeon;

    private double m_yaw;

    private DifferentialDrive m_differentialDrive;

    //Drivetrain kinematics, feed it width between wheels
    private SimpleMotorFeedforward m_feedForward;

    //Drivetrain odometry to keep track of our position on the field
    private DifferentialDriveOdometry m_odometry;

    // private Pose2d m_pose;

    public static final double kNominalVoltageVolts = 12.0;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_targetVelocityRotationsPerSecond;

    //private double m_leftPosition, m_rightPosition;
    private Supplier<Transmission.GearState> m_gearStateSupplier;
    private double m_prevLeftEncoder, m_prevRightEncoder;
    private double m_prevSetOutputTime; 

    private double m_leftVelocity, m_rightVelocity; 

    ShuffleboardTab m_driveTab;

    // -----------------------------------------------------------
    // Initialization
    // -----------------------------------------------------------
    public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {

        m_gearStateSupplier = gearStateSupplier;

        m_pigeon = new Pigeon();
        m_pigeon.resetGyro();

        m_leftLeader = new WPI_TalonFX(RobotMap.kDrivetrainLeftBackTalonFX);
        m_rightLeader = new WPI_TalonFX(RobotMap.kDrivetrainRightBackTalonFX);
        m_leftFollower = new WPI_TalonFX(RobotMap.kDrivetrainLeftFrontTalonFX);
        m_rightFollower = new WPI_TalonFX(RobotMap.kDrivetrainRightFrontTalonFX);

        // Motors
        configmotors();

        // PID values for the talons
        setWheelPIDF();
        
        m_differentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);   

        // Feedforward contraints
        m_feedForward = DrivetrainConstants.kFeedForward;
        
        // Save previous wheel speeds. Start at zero.
        m_prevSpeeds = new DifferentialDriveWheelSpeeds(0,0);

        // Setup odometry to start at position 0,0 (top left of field)
        m_yaw = m_pigeon.getYaw();
        SmartDashboard.putNumber("Initial robot yaw", m_yaw);
    
        Rotation2d initialHeading = new Rotation2d(m_yaw);

        // Start with default Pose2d(0, 0, 0)
        m_odometry = new DifferentialDriveOdometry(initialHeading);

        // Zero the encoders
        resetEncoders();

        setupShuffleboard();
    }        

    public void configmotors() {

        //Setting followers, followers don't automatically followtLeader's inverts so you must set the invert type to FollotLeader
        m_leftFollower.follow(m_leftLeader, FollowerType.PercentOutput);
        m_leftFollower.setInverted(InvertType.FollowMaster);
        m_rightFollower.follow(m_rightLeader, FollowerType.PercentOutput);
        m_rightFollower.setInverted(InvertType.FollowMaster);

        m_rightLeader.setInverted(InvertType.InvertMotorOutput);

        // Configure the motors
        for(TalonFX fx : new TalonFX[] {m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower}){
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

            fx.configOpenloopRamp(0.1);

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
    }

    public void setWheelPIDF() {

        // set the PID values for each individual wheel
        for(TalonFX fx : new TalonFX[] {m_leftLeader, m_rightLeader}){
            
            fx.config_kP(0, DrivetrainConstants.kGainsProfiled.kP, 0);
            fx.config_kI(0, DrivetrainConstants.kGainsProfiled.kI, 0);
            fx.config_kD(0, DrivetrainConstants.kGainsProfiled.kD, 0);
            fx.config_kF(0, DrivetrainConstants.kGainsProfiled.kF, 0);
            // m_talonsMaster.config_IntegralZone(0, 30);
        }
    }

    private void setupShuffleboard() {

        // Create a tab for the Drivetrain
        ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drivetrain");
        m_driveTab.add("Heading Deg.", getHeading())
            .withPosition(0, 0)
            .getEntry();  
        m_driveTab.add("Left Wheel Pos.", getLeftDistanceMeters())
            .withPosition(2, 0)
            .getEntry();  
        m_driveTab.add("Right Wheel Pos.", getRightDistanceMeters())
            .withPosition(4, 0)
            .getEntry(); 
        m_driveTab.add("Left FF", 0)
            .withPosition(2, 4)
            .getEntry();  
        m_driveTab.add("Right FF", 0)
            .withPosition(4, 4)
            .getEntry();               
    }

    // -----------------------------------------------------------
    // Process Logic
    // -----------------------------------------------------------
    @Override
    public void periodic() {
    

        // double leftWheelRotations, rightWheelRotations;

        var gearState = m_gearStateSupplier.get();
        double leftPosition = getLeftDistanceMeters();
        double rightPosition = getRightDistanceMeters();

        double leftEncoderVelocity = m_leftLeader.getSelectedSensorVelocity();
        double rightEncoderVelocity = m_rightLeader.getSelectedSensorVelocity();
        m_leftVelocity = (wheelRotationsToMeters(motorRotationsToWheelRotations(leftEncoderVelocity, gearState)) * 10);
        m_rightVelocity = (wheelRotationsToMeters(motorRotationsToWheelRotations(rightEncoderVelocity, gearState)) * 10);

        // Update the odometry in the periodic block
        m_yaw = m_pigeon.getYaw();
        m_odometry.update(Rotation2d.fromDegrees(m_yaw), leftPosition, rightPosition);

        //Stores current values for next run through
        //m_prevLeftEncoder = leftEncoderCount;
        //m_prevRightEncoder = rightEncoderCount;

        SmartDashboard.putNumber("Left Wheel Position", leftPosition);
        SmartDashboard.putNumber("Right Wheel Position", rightPosition);
        //SmartDashboard.putNumber("Rotations Left Wheel", m_leftWheelRotations);
        //SmartDashboard.putNumber("Rotations Right Wheel", m_rightWheelRotations);
        SmartDashboard.putNumber("Left Wheel Speed", m_leftVelocity);
        SmartDashboard.putNumber("Right Wheel Speed", m_rightVelocity);
        SmartDashboard.putNumber("Robot yaw", m_yaw);
        //SmartDashboard.putNumber("Drivetrain Left encoder", leftEncoderCount);
        //SmartDashboard.putNumber("Drivetrain Right encoder", rightEncoderCount);
    }
    public double metersToWheelRotations(double metersPerSecond) {
        return metersPerSecond / (DrivetrainConstants.kWheelDiameterMeters * Math.PI);
    }

    public double wheelRotationsToEncoderTicks(double wheelRotations, Transmission.GearState gearState) {
        if (gearState == Transmission.GearState.HIGH) {
            return wheelRotations * DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio;
        }
        return wheelRotations * DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio;
    }

    public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
        if (gearState == Transmission.GearState.HIGH) {
            return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio);
        }
        return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio);
    }

    public double wheelRotationsToMeters(double wheelRotations) {
        return DrivetrainConstants.kWheelDiameterMeters * Math.PI * wheelRotations;
    }

    // -----------------------------------------------------------
    // Control Input
    // -----------------------------------------------------------
    public void drive(DoubleSupplier move, DoubleSupplier rotate){
        drive(move.getAsDouble(), rotate.getAsDouble(), true);
    }

    public void drive(double move, double rotate, boolean squaredInputs){
        m_differentialDrive.arcadeDrive(move, rotate, squaredInputs);
    }

    public void drive(double move, double rotate){
        drive(move, rotate, true);
    }

    public void setDriveTrainVoltage(double leftVolts, double rightVolts) {
        m_leftLeader.set(ControlMode.PercentOutput, leftVolts/12);
        m_rightLeader.set(ControlMode.PercentOutput, rightVolts/12);
        m_differentialDrive.feed();
    }

    public void setOutputMetersPerSecond(double leftMetersPerSecond, double rightMetersPerSecond) {
        

        // Calculate feedforward for the left and right wheels.
        double leftFeedForward = m_feedForward.calculate(leftMetersPerSecond);
        double rightFeedForward = m_feedForward.calculate(rightMetersPerSecond);

        SmartDashboard.putNumber("left feed forward", leftFeedForward);
        SmartDashboard.putNumber("right feed forward", rightFeedForward);
        
        // Convert meters per second to rotations per second
        var gearState = m_gearStateSupplier.get();
        double leftVelocityTicksPerSec = wheelRotationsToEncoderTicks(metersToWheelRotations(leftMetersPerSecond), gearState);
        double rightVelocityTicksPerSec = wheelRotationsToEncoderTicks(metersToWheelRotations(rightMetersPerSecond), gearState);

        SmartDashboard.putNumber("left velocity ticks per second", leftVelocityTicksPerSec);
        SmartDashboard.putNumber("right velocity ticks per second", rightVelocityTicksPerSec);

        m_leftLeader.set(ControlMode.Velocity, leftVelocityTicksPerSec/10.0, DemandType.ArbitraryFeedForward, leftFeedForward);
        m_rightLeader.set(ControlMode.Velocity, rightVelocityTicksPerSec/10.0, DemandType.ArbitraryFeedForward, rightFeedForward);

        m_differentialDrive.feed();
    }

    public void stopDrivetrain() {
        setDriveTrainVoltage(0.0, 0.0);
    }

    public void setMaxOutput(double maxOutput) {
        m_differentialDrive.setMaxOutput(maxOutput);
    }

    public void disableMotorSafety(){
        m_differentialDrive.setSafetyEnabled(false);
    }

    public void enableMotorSafety(){
        m_differentialDrive.setSafetyEnabled(true);
    }

    public void feedWatchdog(){
        m_differentialDrive.feed();
    }

    public void zeroGyro(){
        m_pigeon.resetGyro();
    }

    public void resetEncoders(){
        m_leftLeader.setSelectedSensorPosition(0);
        m_rightLeader.setSelectedSensorPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        // zeroGyro();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void setPIDSlot(int slot) {
        int PID_PRIMARY = 0;
        m_leftLeader.selectProfileSlot(slot, PID_PRIMARY);
        m_rightLeader.selectProfileSlot(slot, PID_PRIMARY);
    }

    // -----------------------------------------------------------
    // System State
    // -----------------------------------------------------------

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftVelocity, m_rightVelocity);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public double getLeftVoltage(){
        return m_leftLeader.getMotorOutputVoltage();
    }

    public double getRightVoltage(){
        return m_rightLeader.getMotorOutputVoltage();
    }

    // Gyro readings
    public double getHeading() {
        return m_pigeon.getYaw();
    }

    public double getLeftDistanceMeters() {
        var gearState = m_gearStateSupplier.get();
        double leftEncoderCount = m_leftLeader.getSelectedSensorPosition();
        double leftWheelRotations = motorRotationsToWheelRotations(leftEncoderCount, gearState);
        double leftPosition = wheelRotationsToMeters(leftWheelRotations);
        return leftPosition;

    }

    public double getRightDistanceMeters() {
        var gearState = m_gearStateSupplier.get();
        double rightEncoderCount = m_rightLeader.getSelectedSensorPosition();
        double rightWheelRotations = motorRotationsToWheelRotations(rightEncoderCount, gearState);
        double rightPosition = wheelRotationsToMeters(rightWheelRotations);
        return rightPosition;
    }

    public double getAvgDistanceMeters(){
        return (getLeftDistanceMeters() + getRightDistanceMeters()) /2;
    }


    // Required methods for SmartSubsystem
    public double getDistance(){
        return 0;
    }
    public double getVelocity(){
        return 0;
    }
    public boolean atReference(){
        return true;
    }

    // -----------------------------------------------------------
    // Testing and Configuration
    // -----------------------------------------------------------
}
