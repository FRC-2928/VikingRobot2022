package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CANBusIDs {
        // Drivetrain, right side
        public static final int kDrivetrainRightBackTalonFX = 0;
        public static final int kDrivetrainRightFrontTalonFX = 1;

        //Shooter
        public static final int kFlywheelTalonLower = 2;
        public static final int kFlywheelTalonUpper = 3;

        //Sensors
        public static final int kPigeonIMU = 0;

        //Intake
        public static final int kIntakeMotor = 4;
        
        //Climber
        public static final int kClimberMotor = 5; 
        public static final int kClimberMotor2 = 8;
        
        //Turret
        public static final int kTurretTalonSRX = 9;

        //Feeder
        public static final int kLeftFeederMotor = 11;
        public static final int kRightFeederMotor = 10;

        // Drivetrain, left side
        public static final int kDrivetrainLeftFrontTalonFX = 14;
        public static final int kDrivetrainLeftBackTalonFX = 15;       
    }

    public static final class PneumaticIDs {
        //Solenoids
        public static final int kDrivetrainShiftSolenoid = 0;
        public static final int kRampSolenoid = 1;
        public static final int kClimberSolenoid = 2;

        public static final int kIntakeSolenoid = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kDriver2ControllerPort = 2;
    }

    public static final class DrivetrainConstants{

        // kS (static friction), kV (velocity), and kA (acceleration)
        public static final double ksVolts = 0.6024;
        public static final double kvVoltSecondsPerMeter = 0.21907;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0096252;

        // Feedforward contraints          
       public static final SimpleMotorFeedforward kFeedForward = 
       new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

        public static final double kTrackWidthMeters = 0.7; //Placeholder
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final double k_MaxVolts = 10;    
        public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                kFeedForward,
                kDriveKinematics,
                k_MaxVolts);    

        public static final boolean kGyroReversed = true;

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.1015;

        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kUnitsPerRevolution = 2048;

        public static final double kHighGearRatio = 4.4;
        public static final double kLowGearRatio = 8.82;

        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAccelMetersPerSecondSquared = 2.0;

        public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints =
            new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond, kMaxAccelMetersPerSecondSquared);
        
        // PID Constants
        // The WPILib feedforward has kS (static friction), kV (velocity), and kA (acceleration) terms 
        // whereas the Talon SRX / Spark MAX kF is only a kV (velocity) feedforward.
        //                                                  kp,  ki, kd,  kf, iz,  peak output
        public static final Gains kGainsProfiled = new Gains(0.16,  0,   0,   0,   0,  1.00);   

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
         * Not all set of Gains are used in this project and may be removed as desired.
         * 
         * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
        public final static Gains kGainsDistance = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
        public final static Gains kGainsTurning = new Gains( 0.10, 0.0,  0.0, 0.0,            200,  1.00 );
        public final static Gains kGainsVelocity = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
        public final static Gains kGainsMotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/20660.0,  400,  1.00 );
    }

    public static final class AutoConstants {
    
        // Setup trajectory constraints
        public static final TrajectoryConfig kTrajectoryConfig =
        new TrajectoryConfig(DrivetrainConstants.kMaxSpeedMetersPerSecond, 
                             DrivetrainConstants.kMaxAccelMetersPerSecondSquared)
            .setKinematics(DrivetrainConstants.kDriveKinematics)
            .addConstraint(DrivetrainConstants.kAutoVoltageConstraint);
            // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;
    }
    
    public static final class FlywheelConstants {

        public static final double ksVolts = 0.6024;
        public static final double kvVoltSecondsPerRadian = 0.00021907;
        public static final double kaVoltSecondsSquaredPerRadian = 0.000096252;

        public static final LinearSystem<N1, N1, N1> kFlywheelLinearSystem = 
            LinearSystemId.identifyVelocitySystem(kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
        
        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
         * 
         * 	                                    			    kP 	 kI     kD   kF    Iz    PeakOut */
        public final static Gains kGainsVelocity1  = new Gains( 0.1, 0.0001, 0, .047,  300,  1.00);

        public final static Gains kGainsVelocity2  = new Gains( .15,.0001, 0, .049,  300,  1.00);
        public final static Gains kGainsVelocity3  = new Gains( .08,  .0001, 0, .045,  300,  1.00);
        public final static Gains kGainsVelocity4 = new Gains(.2, .0001, 0, .051, 300, 1.0);

        public static final double kGearRatio = 1;

        public static final double kEncoderCPR = 2048;
        
        //velocity in ticks per 100 ms
        public static final double kIdealVelocity = 6200;
    }

    

    public static final class TurretConstants {
        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to TalonSRX at 100%, 6800 represents Velocity units at 100% output
         * Not all set of Gains are used in this project and may be removed as desired.
         * 
         * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
        public final static Gains kGainsTurret = new Gains( 0.01, 0.0,  0.0, 0.0,            100,  0.50 );

        public static final double kTurretGearRatio = 9.08;
        public static final double kTurretDegreesPerRotation = 360;

        public static final double kTurretLeftLimit = 120;
        public static final double kTurretRightLimit = -120;

        public static final double kTurretMaxTicks = 1;

        public static final double kTurretTicksPerDegree = 61;
        
        public static final int kEncoderCPR = 4096;
        public static final double kGearRatio = (50/9);

        public static final double ksVolts = 0.06024;
        public static final double kvVoltSecondsPerRadian = 0.005;
        public static final double kaVoltSecondsSquaredPerRadian = 0.000096252;

        public static final LinearSystem<N2, N1, N1> kTurretLinearSystem = 
            LinearSystemId.identifyPositionSystem(kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
        
        public static final double kMaxSpeedRotationsPerSecond = 50.0;
        public static final double kMaxAccelRotationsPerSecondSquared = 50.0;
        public static final double kangleToleranceDegrees = 5.0;

        public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints =
            new TrapezoidProfile.Constraints(kMaxSpeedRotationsPerSecond, kMaxAccelRotationsPerSecondSquared);    
    }

    public static final class ClimberConstants {

        public static final Gains kGainsClimber = new Gains(0,  0,   0,   0,   0,  1.00);

        public static final double kGearRatio = 1;

        public static final double kEncoderCPR = 4096;

        public static final double kClimberPower = 0.3;
    }

    public static final class IntakeConstants {
        public static final double ksVolts = 0.6024;
        public static final double kvVoltSecondsPerMeter = 0.21907;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0096252;

        public static final LinearSystem<N1, N1, N1> kIntakeSystem =
            LinearSystemId.identifyVelocitySystem(IntakeConstants.kvVoltSecondsPerMeter, 
                                          IntakeConstants.kaVoltSecondsSquaredPerMeter);

        public static double IntakekP = 0.055;
        public static double IntakekI = 0;
        public static double IntakekD = 0.5;
        public static double IntakekF = 0.4;
		public static double kClimberErrorThreshold = 0.5; // 5 cm

        public static double kIntakeLowSpeed = .2;
        public static double kIntakeSpeed = .5;      
        public static double kFeederSpeed = .37;
        public static double kFeederHighSpeed = .8;

        public static int kLeftIntakeSensor = 8;
        public static int kRightIntakeSensor = 9;
    }

    public static final class LimelightConstants{

        // //Limelight names
        public static final String kDriverLimelight = "limelight-driver";
        public static final String kTurretLimelight = "limelight-turret";

        public static final double kHighLimelightMountAngle = 17.5;
        public static final double kHighLimelightHeight = 37.5;
        public static final double kHighGoalHeight = 90;
    }
    public static final class ConversionConstants{
        public static final double kMetersToFeet = 3.281;
    
        // Flywheel
        public static final double kFlywheelEncoderTicksPerRotation = 2048;
        public static final double kFlywheelGearRatio = 1;

        // Hood
        public static final double kHoodEncoderTicksPerRotation = 4096;
        public static final double kHoodGearRatio = (10*5) * (60.0/24.0); 

        // Turret
        public static final double kTurretGearRatio = 169.155; 
        public static final double kTurretDegreesPerRotation = 360; 
    }
}