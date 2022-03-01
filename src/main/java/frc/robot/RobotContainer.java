package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Transmission;
import frc.robot.subsystems.Turret;
import frc.robot.commands.ClimberCommands.ExtendClimberBars;
import frc.robot.commands.ClimberCommands.RetractClimberBars;
import frc.robot.commands.DrivetrainCommands.DriveDistanceProfiled;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.commands.FlywheelCommands.DecrementFlywheel;
import frc.robot.commands.FlywheelCommands.IncrementFlywheel;
import frc.robot.commands.FlywheelCommands.ToggleFlywheel;
import frc.robot.commands.IntakeCommands.CloseRamp;
import frc.robot.commands.IntakeCommands.EjectBall;
import frc.robot.commands.IntakeCommands.OpenRamp;
import frc.robot.commands.IntakeCommands.ShootBall;
import frc.robot.commands.IntakeCommands.ToggleFeederMotor;
import frc.robot.commands.IntakeCommands.ToggleIntakeMotor;
import frc.robot.commands.TurretCommands.AutoTrackingTurret;
import frc.robot.commands.TurretCommands.MoveTurret;
import frc.robot.commands.TurretCommands.TurnTurretToTarget;

public class RobotContainer {

  // The Robot's Subsystems
  private final Transmission m_transmission = new Transmission();
  private final Drivetrain m_drivetrain = new Drivetrain(m_transmission::getGearState);
  // private final Pigeon m_pigeon = new Pigeon();
  private final Turret m_turret = new Turret(m_drivetrain);
  private final Intake m_intake = new Intake(DriverStation.getAlliance());
  private final Flywheel m_flywheel = new Flywheel();
  private final Climber m_climber = new Climber();
  

  // XBox Controllers
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final DriverOI m_driverOI = new DriverOI(m_driverController);
  private final OperatorOI m_operatorOI = new OperatorOI(m_operatorController);
  
  // Shuffleboard 
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Default option is set in configureDrivetrain()
    SmartDashboard.putData(m_autoChooser);  
    
    // Configure default commands, button bindings, and shuffleboard
    configureSubsystems();
    
  }

  public void onAutoInit(){
    new InstantCommand(m_drivetrain::zeroGyro);
     m_intake.setAllianceColor(DriverStation.getAlliance());
    // new TrackTargetCommand(m_turret, m_drivetrain, m_turretLimelight).schedule();
  }

  public void onTeleopInit() {  
    m_intake.setAllianceColor(DriverStation.getAlliance());
  }


  /**
   * Configure all subsystems with their default command, button commands,
   * and Shuffleboard output
   */
  private void configureSubsystems() {
    configureDrivetrain();
    configureTurret();
    configureIntake();
    configureFlywheel();
    configureClimber();
  }

  /**
   * Configure Drivetrain
   */
  public void configureDrivetrain() {
    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain.drive(m_driverOI.getMoveSupplier(), m_driverOI.getRotateSupplier()),
            m_drivetrain));

    // Configure button commands
    m_driverOI.getShiftLowButton().whenPressed(new InstantCommand(m_transmission::setLow, m_transmission));
    m_driverOI.getShiftHighButton().whenPressed(new InstantCommand(m_transmission::setHigh, m_transmission));
    m_operatorOI.getPrintButton().whenPressed(new PrintCommand("Print from Operator"));
    //   m_driverOI.getResetEncodersButton().whenPressed(new InstantCommand(m_drivetrain::resetEncoders, m_drivetrain));

    // Configure Shuffleboard commands
    m_autoChooser.setDefaultOption("Calibrate Robot", new RunRamseteTrajectory(m_drivetrain, calibrateTrajectory()));
    m_autoChooser.addOption("Red 1", new RunRamseteTrajectory(m_drivetrain, loadTrajectory("Red1")));
    m_autoChooser.addOption("Figure 8", new RunRamseteTrajectory(m_drivetrain, loadTrajectory("Figure8")));
    m_autoChooser.addOption("Straight", new RunRamseteTrajectory(m_drivetrain, loadTrajectory("Straight")));
    m_autoChooser.addOption("Navigate Cones", new RunRamseteTrajectory(m_drivetrain, navigateConesTrajectory()));
    m_autoChooser.addOption("Drive Distance PID", new DriveDistanceProfiled(3.0, m_drivetrain));
    m_autoChooser.addOption("Reverse Distance PID", new DriveDistanceProfiled(-3.0, m_drivetrain));
  }

  /**
   * Configure Turret
   */
  public void configureTurret() {
    
    // Configure default commands
    // m_turret.setDefaultCommand(new TurnTurretToTarget(m_turret));
    m_turret.setDefaultCommand(
        new RunCommand(() -> m_turret.rotateTurret(m_operatorOI.getRotateTurretSupplier()),
            m_turret));

    // Configure button commands
    m_driverOI.getTurnTurretLeftButton().whileHeld(new MoveTurret(m_turret, -1));
    m_driverOI.getTurnTurretRightButton().whileHeld(new MoveTurret(m_turret, 1));
    m_driverOI.getTurnTurretToTargetButton().whileHeld(new AutoTrackingTurret(m_turret));
    // m_driverOI.getTurnTurretToTargetButton().whileHeld(new TurnTurretToTarget(m_turret));


    // m_driverOI.getToggleIntakeMotorButton().whenPressed(new PrintCommand("Print from Driver"));
    // Configure Shuffleboard commands    
  }

  /**
   * Configure Intake
   */
  public void configureIntake() {
    // Configure default commands
    // m_intake.setDefaultCommand(new RunCommand(m_intake::startMotors, m_intake));

    // Configure button commands
    m_driverOI.getToggleIntakeMotorButton().whenPressed(new ToggleIntakeMotor(m_intake));
    m_driverOI.getToggleFeederMotorButton().whenPressed(new ToggleFeederMotor(m_intake));

    // Configure Shuffleboard commands
    m_intake.getCommandsLayout().add(new ToggleIntakeMotor(m_intake)); 
    m_intake.getCommandsLayout().add(new ToggleFeederMotor(m_intake)); 
    // m_intake.getCommandsLayout().add(new PrintCommand("Toggle feeder button")); 
    // m_intake.getCommandsLayout().add(new InstantCommand(m_intake::triggerCloseIntakeSwitchSim, m_intake));
    m_intake.getCommandsLayout().add(new ShootBall(m_intake)); 
    m_intake.getCommandsLayout().add(new EjectBall(m_intake));
    m_intake.getCommandsLayout().add(new OpenRamp(m_intake));
    m_intake.getCommandsLayout().add(new CloseRamp(m_intake));
  }

  /**
   * Configure Flywheel
   */
  public void configureFlywheel() {

    // Configure default commands

    // Configure button commands
    m_driverOI.getIncrementFlywheelButton().whileHeld(new IncrementFlywheel(m_flywheel));
    m_driverOI.getDecrementFlywheelButton().whileHeld(new DecrementFlywheel(m_flywheel));
    m_driverOI.getToggleFlywheelButton().whenPressed(new ToggleFlywheel(m_flywheel));

    // Configure Shuffleboard commands
    m_flywheel.getCommandsLayout().add(new DecrementFlywheel(m_flywheel));
    m_flywheel.getCommandsLayout().add(new IncrementFlywheel(m_flywheel));
    m_flywheel.getCommandsLayout().add(new ToggleFlywheel(m_flywheel));
    
  }

  /**
   * Configure Climber
   */
  public void configureClimber() {
    // Configure default commands

    // Configure button commands
    m_operatorOI.getExtendClimber().whileHeld(new ExtendClimberBars(m_climber));
    m_operatorOI.getRetractClimber().whileHeld(new RetractClimberBars(m_climber));
    m_operatorOI.getTiltForward().whenPressed(new InstantCommand(m_climber::tiltForward,m_climber));
    m_operatorOI.getTiltBack().whenPressed(new InstantCommand(m_climber::tiltBack,m_climber));
  }


  public Trajectory calibrateTrajectory() {
    
    // Note that all coordinates are in meters, and follow NWU conventions.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 4, new Rotation2d(0)),
        List.of(
            new Translation2d(1.0, 4.0)
        ),
        new Pose2d(3.0, 4.0, new Rotation2d(0)), // left
        AutoConstants.kTrajectoryConfig);

    return trajectory;
  }

  public Trajectory navigateConesTrajectory() {

    // Note that all coordinates are in meters, and follow NWU conventions.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 4, new Rotation2d(0)),
        List.of(
            new Translation2d(3.0, 5.0),
            new Translation2d(6.0, 3.0),
            new Translation2d(9.0, 5.0),
            new Translation2d(12.0, 4.0),
            new Translation2d(9.0, 3.0),
            new Translation2d(6.0, 5.0),
            new Translation2d(3.0, 3.0)
        ),
        new Pose2d(0, 4, new Rotation2d(180)), // left
        AutoConstants.kTrajectoryConfig);

    return trajectory;
  }

  public Trajectory loadTrajectory(String trajectoryJSON) {
    Trajectory trajectory = new Trajectory();

    try{
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("output", trajectoryJSON + ".wpilib.json"));
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open Trajectory:" + trajectoryJSON, ex.getStackTrace());
      }
      return trajectory;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();          
  }

  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }

  // public boolean getButtonA() {
  //   return m_driverController.getAButton();
  // }
    
  }
