package frc.robot.subsystems;

import java.lang.reflect.Field;
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

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants;

// Imports for Simulation
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
   * DrivetrainSubsystem handles all subsystem level logic for the drivetrain.
   * Possibly also Ramsete idfk I haven't finished this class yet.
   */
import frc.robot.subsystems.Pigeon;

public class Drivetrain extends SubsystemBase {

    private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftBackTalonFX);
    private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightBackTalonFX);
    private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftFrontTalonFX);
    private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightFrontTalonFX);

    private WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.CANBusIDs.kPigeonIMU);

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
    private final Field2d m_field2d = new Field2d();

    NetworkTableEntry m_leftFFEntry, m_rightFFEntry, m_headingEntry;
    NetworkTableEntry m_leftWheelPositionEntry, m_rightWheelPositionEntry;
    NetworkTableEntry m_odometryXEntry, m_odometryYEntry, m_odometryHeadingEntry;


    // ------ Simulation classes to help us simulate our robot ---------
    TalonFXSimCollection m_leftDriveSim = m_leftLeader.getSimCollection();
    TalonFXSimCollection m_rightDriveSim = m_rightLeader.getSimCollection();
    private final BasePigeonSimCollection m_pigeonSim = m_pigeon.getSimCollection();

    private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
        LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);

    // Simulation model of the drivetrain 
    private final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(
          m_drivetrainSystem, DCMotor.getFalcon500(2), 
          DrivetrainConstants.kLowGearRatio, 
          DrivetrainConstants.kTrackWidthMeters, 
          DrivetrainConstants.kWheelDiameterMeters, 
          null);

    // -----------------------------------------------------------
    // Initialization
    // -----------------------------------------------------------
    public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {

        m_gearStateSupplier = gearStateSupplier;

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

        // Zero the encoders and gyro
        resetEncoders();
        zeroGyro();

        // Start with default Pose2d(0, 0, 0)
        m_odometry = new DifferentialDriveOdometry(initialHeading);

        m_field2d.setRobotPose(getPose());
        SmartDashboard.putData("Field", m_field2d);

        setupShuffleboard();
    }        

    public void configmotors() {


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

            fx.configOpenloopRamp(0);

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
        
        //Setting followers, followers don't automatically followtLeader's inverts so you must set the invert type to FollotLeader
        m_leftFollower.follow(m_leftLeader, FollowerType.PercentOutput);
        m_leftFollower.setInverted(InvertType.FollowMaster);
        m_rightFollower.follow(m_rightLeader, FollowerType.PercentOutput);
        m_rightFollower.setInverted(InvertType.FollowMaster);

        m_rightLeader.setInverted(InvertType.InvertMotorOutput);
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
        m_headingEntry = m_driveTab.add("Heading Deg.", getRotation().getDegrees())
            .withWidget(BuiltInWidgets.kGraph)      
            .withSize(3,3)
            .withPosition(0, 0)
            .getEntry();  
        m_leftWheelPositionEntry = m_driveTab.add("Left Wheel Pos.", getLeftDistanceMeters())
            .withWidget(BuiltInWidgets.kGraph)      
            .withSize(3,3)  
            .withPosition(4, 0)
            .getEntry();  
        m_rightWheelPositionEntry = m_driveTab.add("Right Wheel Pos.", getRightDistanceMeters())
            .withWidget(BuiltInWidgets.kGraph)      
            .withSize(3,3)
            .withPosition(7, 0)
            .getEntry(); 
        m_leftFFEntry=m_driveTab.add("Left FF", 0)
            .withWidget(BuiltInWidgets.kGraph)      
            .withSize(3,3)            
            .withPosition(4, 3)
            .getEntry();  
        m_rightFFEntry=m_driveTab.add("Right FF", 0)
            .withWidget(BuiltInWidgets.kGraph)            
            .withSize(3,3)
            .withPosition(7, 3)
            .getEntry();   
            
        ShuffleboardTab m_odometryTab = Shuffleboard.getTab("Odometry");    
        m_odometryXEntry = m_odometryTab.add("X Odometry", 0)
            .withWidget(BuiltInWidgets.kGraph)            
            .withSize(2,2)
            .withPosition(7, 0)
            .getEntry();
        m_odometryYEntry = m_odometryTab.add("Y Odometry", 0)
            .withWidget(BuiltInWidgets.kGraph)            
            .withSize(2,2)
            .withPosition(9, 0)
            .getEntry();
        m_odometryHeadingEntry = m_odometryTab.add("Heading Odometry", 0)
            .withWidget(BuiltInWidgets.kGraph)            
            .withSize(2,2)
            .withPosition(8, 3)
            .getEntry();
                   
    }

    // -----------------------------------------------------------
    // Process Logic
    // -----------------------------------------------------------
    @Override
    public void periodic() {

        publishTelemetry();   
        
    }

    public void publishTelemetry() {
        double leftPosition = getLeftDistanceMeters();
        double rightPosition = getRightDistanceMeters();

        double leftEncoderVelocity = m_leftLeader.getSelectedSensorVelocity();
        double rightEncoderVelocity = m_rightLeader.getSelectedSensorVelocity();
        m_leftVelocity = ((encoderTicksToMeters(leftEncoderVelocity)) * 10);
        m_rightVelocity = ((encoderTicksToMeters(rightEncoderVelocity)) * 10);

        // Update the odometry for either real or simulated robot
        m_odometry.update(getRotation(), leftPosition, rightPosition);
   
        m_headingEntry.setDouble(m_yaw);
        m_field2d.setRobotPose(getPose());

        m_odometryXEntry.setDouble(m_odometry.getPoseMeters().getX());
        m_odometryYEntry.setDouble(m_odometry.getPoseMeters().getY());
        m_odometryHeadingEntry.setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Left Wheel Position", leftPosition);
        SmartDashboard.putNumber("Right Wheel Position", rightPosition);
        m_leftWheelPositionEntry.setDouble(leftPosition);
        m_rightWheelPositionEntry.setDouble(rightPosition);
        SmartDashboard.putNumber("Left Wheel Speed", m_leftVelocity);
        SmartDashboard.putNumber("Right Wheel Speed", m_rightVelocity);
        SmartDashboard.putNumber("Robot yaw", getRotation().getDegrees());
    }

    // Meters to encoder ticks
    public double metersToEncoderTicks(double meters) {
        var gearState = m_gearStateSupplier.get();
        double wheelRotations = metersToWheelRotations(meters);
        return wheelRotationsToEncoderTicks(wheelRotations, gearState);
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

    // Encoder ticks to meters
    public double encoderTicksToMeters(double encoderTicks) {
        var gearState = m_gearStateSupplier.get();
        double wheelRotations = motorRotationsToWheelRotations(encoderTicks, gearState);
        return wheelRotationsToMeters(wheelRotations);
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
        SmartDashboard.putNumber("Output", rotate);
        m_differentialDrive.arcadeDrive(move, -.8* rotate, squaredInputs);
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

        SmartDashboard.putNumber("left meters per sec", leftMetersPerSecond);
        SmartDashboard.putNumber("right meters per sec", rightMetersPerSecond);

        m_rightFFEntry.setDouble(rightFeedForward);
        m_leftFFEntry.setDouble(leftFeedForward);
        
        // Convert meters per second to encoder ticks per second
        // TODO use metersToEncoderTicks(double meters)
        var gearState = m_gearStateSupplier.get();
        double leftVelocityTicksPerSec = wheelRotationsToEncoderTicks(metersToWheelRotations(leftMetersPerSecond), gearState);
        double rightVelocityTicksPerSec = wheelRotationsToEncoderTicks(metersToWheelRotations(rightMetersPerSecond), gearState);

        SmartDashboard.putNumber("left velocity ticks per second", leftVelocityTicksPerSec);
        SmartDashboard.putNumber("right velocity ticks per second", rightVelocityTicksPerSec);

        m_leftLeader.set(ControlMode.Velocity, 
                        leftVelocityTicksPerSec/10.0, 
                        DemandType.ArbitraryFeedForward, 
                        leftFeedForward / DrivetrainConstants.k_MaxVolts);
        m_rightLeader.set(ControlMode.Velocity, 
                        rightVelocityTicksPerSec/10.0, 
                        DemandType.ArbitraryFeedForward, 
                        rightFeedForward / DrivetrainConstants.k_MaxVolts);

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
        m_pigeon.reset();
    }

    public void resetEncoders(){
        m_leftLeader.setSelectedSensorPosition(0);
        m_rightLeader.setSelectedSensorPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getRotation());       
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

    public Rotation2d getRotation(){
        m_yaw = m_pigeon.getYaw();
        if (RobotBase.isReal()) {
           return (Rotation2d.fromDegrees(Math.IEEEremainder(m_yaw, 360.0d) * -1.0d)); 
        } else {
            return (Rotation2d.fromDegrees(m_yaw));
        }       
    }

    public double getLeftDistanceMeters() {
        return encoderTicksToMeters(m_leftLeader.getSelectedSensorPosition());
    }

    public double getRightDistanceMeters() {        
        return encoderTicksToMeters(m_rightLeader.getSelectedSensorPosition());
    }

    public double getAvgDistanceMeters(){
        return (getLeftDistanceMeters() + getRightDistanceMeters()) /2;
    }


    // -----------------------------------------------------------
    // Simulation
    // -----------------------------------------------------------
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated Talon FXs */
        m_leftDriveSim.setBusVoltage(RobotController.getInputVoltage());
        m_rightDriveSim.setBusVoltage(RobotController.getInputVoltage());
          
        m_drivetrainSimulator.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                                        -m_rightDriveSim.getMotorOutputLeadVoltage());
    
        /*
         * Advance the model by 20 ms. Note that if you are running this
         * subsystem in a separate thread or have changed the nominal
         * timestep of TimedRobot, this value needs to match it.
         */
        m_drivetrainSimulator.update(0.02);
    
        /*
         * Update all of the sensors.
         *
         * Since WPILib's simulation class is assuming +V is forward,
         * but -V is forward for the right motor, we need to negate the
         * position reported by the simulation class. Basically, we
         * negated the input, so we need to negate the output.
         */
        m_leftDriveSim.setIntegratedSensorRawPosition(
                        (int)metersToEncoderTicks(
                            m_drivetrainSimulator.getLeftPositionMeters()
                        ));
        m_leftDriveSim.setIntegratedSensorVelocity(
                        (int)metersToEncoderTicks(
                            m_drivetrainSimulator.getLeftVelocityMetersPerSecond() / 10
                        ));
        m_rightDriveSim.setIntegratedSensorRawPosition(
                        (int)metersToEncoderTicks(
                            -m_drivetrainSimulator.getRightPositionMeters()
                        ));
        m_rightDriveSim.setIntegratedSensorVelocity(
                        (int)metersToEncoderTicks(
                            -m_drivetrainSimulator.getRightVelocityMetersPerSecond() / 10
                        ));

        m_pigeonSim.setRawHeading(m_drivetrainSimulator.getHeading().getDegrees());
      }

}
