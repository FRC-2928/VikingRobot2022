package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OIConstants;

import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Transmission;
import frc.robot.commands.RunRamseteTrajectory;

public class RobotContainer {

  // The robot's subsystems

  private final Transmission m_transmission = new Transmission();
  private final Drivetrain m_drivetrain = new Drivetrain(m_transmission::getGearState);
  
  
  
  private final Pigeon m_pigeon = new Pigeon();
  
  private final DriverOI m_driverOI;
  
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  

  private final SendableChooser<Command> m_autoChooser;

  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Do Nothing", RunRamseteTrajectory());
    
    SmartDashboard.putData(m_autoChooser);

    m_driverOI = new DriverOI(m_driverController);
    


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain.drive(m_driverOI.getMoveSupplier(), m_driverOI.getRotateSupplier()),
            m_drivetrain));
  }

  public void onAutoInit(){
    new InstantCommand(m_pigeon::resetGyro);
    // new TrackTargetCommand(m_turret, m_drivetrain, m_turretLimelight).schedule();
  }

  public void onTeleopInit() {

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    configureDrivetrainButtons();

  }

  public void configureDrivetrainButtons() {
    m_driverOI.getShiftLowButton().whenPressed(new InstantCommand(m_transmission::setLow, m_transmission));

    m_driverOI.getShiftHighButton().whenPressed(new InstantCommand(m_transmission::setHigh, m_transmission));
  }

  

  

  

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    
     //  default:
       return new WaitCommand(15); 
        
      
    }

    
    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
    //                                    DrivetrainConstants.kvVoltSecondsPerMeter,
    //                                    DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
    //         DrivetrainConstants.kDriveKinematics,
    //         10);

    // //Create config for trajectory
    // TrajectoryConfig config =
    //   new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    //                         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //       // Add kinematics to ensure max speed is actually obeyed
    //       .setKinematics(DrivetrainConstants.kDriveKinematics)
    //       // Apply the voltage constraint
    //       .addConstraint(autoVoltageConstraint);

    // // Get a trajectory
    // Test1Trajectory trajectory1 = new Test1Trajectory(config);

    // RamseteTrajectoryCommand trajectoryCommand = new RamseteTrajectoryCommand(m_drivetrain, trajectory1.getTrajectory());

    // // Run path following command, then stop at the end.
    // return trajectoryCommand.andThen(() -> m_drivetrain.stopDrivetrain());
  }
