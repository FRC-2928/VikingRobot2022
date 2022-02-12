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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Transmission;
import frc.robot.subsystems.Turret;
import frc.robot.commands.DrivetrainCommands.DriveDistanceProfiled;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.commands.DrivetrainCommands.TurnToTarget;
import frc.robot.commands.FlywheelCommands.DecrementFlywheel;
import frc.robot.commands.FlywheelCommands.IncrementFlywheel;
import frc.robot.commands.FlywheelCommands.ToggleFlywheel;
import frc.robot.commands.IntakeCommands.EjectBall;
import frc.robot.commands.IntakeCommands.ShootBall;
import frc.robot.commands.IntakeCommands.ToggleFeederMotor;
import frc.robot.commands.IntakeCommands.ToggleIntakeMotor;
import frc.robot.commands.TurretCommands.MoveTurret;

public class RobotContainer {

  // The robot's subsystems

  private final Transmission m_transmission = new Transmission();
  private final Drivetrain m_drivetrain = new Drivetrain(m_transmission::getGearState);
  private final Turret m_turret = new Turret();
  private final Intake m_intake = new Intake();
  private final Flywheel m_flywheel = new Flywheel();

  
  
  
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
    m_autoChooser.setDefaultOption("Calibrate Robot", new RunRamseteTrajectory(m_drivetrain, calibrateTrajectory()));
    m_autoChooser.addOption("Red 1", new RunRamseteTrajectory(m_drivetrain, loadTrajectory("Red1")));
    m_autoChooser.addOption("Figure 8", new RunRamseteTrajectory(m_drivetrain, loadTrajectory("Figure8")));
    m_autoChooser.addOption("Straight", new RunRamseteTrajectory(m_drivetrain, loadTrajectory("Straight")));
    m_autoChooser.addOption("Navigate Cones", new RunRamseteTrajectory(m_drivetrain, navigateConesTrajectory()));
    m_autoChooser.addOption("Drive Distance PID", new DriveDistanceProfiled(3.0, m_drivetrain));
    m_autoChooser.addOption("Reverse Distance PID", new DriveDistanceProfiled(-3.0, m_drivetrain));
    
    
    
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

    m_turret.setDefaultCommand(new RunCommand(() -> m_turret.targetHorizontalOffset(), m_turret)); 
    
    m_intake.getCommandsLayout().add(new ToggleIntakeMotor(m_intake)); 
    m_intake.getCommandsLayout().add(new ToggleFeederMotor(m_intake)); 
    m_intake.getCommandsLayout().add(new InstantCommand(m_intake::triggerActivateIntakeSwitchSim, m_intake));
    m_intake.getCommandsLayout().add(new ShootBall(m_intake)); 
    m_intake.getCommandsLayout().add(new EjectBall(m_intake));

    m_flywheel.getCommandsLayout().add(new DecrementFlywheel(m_flywheel));
    m_flywheel.getCommandsLayout().add(new IncrementFlywheel(m_flywheel));
    m_flywheel.getCommandsLayout().add(new ToggleFlywheel(m_flywheel));
  }

  public void onAutoInit(){
    new InstantCommand(m_pigeon::resetGyro);
    // new TrackTargetCommand(m_turret, m_drivetrain, m_turretLimelight).schedule();
  }

  public void onTeleopInit() {
    
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

    m_driverOI.getTurnToTargetButton().whileHeld(new TurnToTarget(m_drivetrain, m_turret));

    m_driverOI.getToggleIntakeMotorButton().whenPressed(new ToggleIntakeMotor(m_intake));

    m_driverOI.getToggleFeederMotorButton().whenPressed(new ToggleFeederMotor(m_intake));

    m_driverOI.getTurnTurretLeftButton().whenHeld(new MoveTurret(m_turret, -1));
    m_driverOI.getTurnTurretRightButton().whenHeld(new MoveTurret(m_turret, 1));
  }

  

  //added 1/19/21 STILL NOT TESTED, should reset the encoders in theory
  // public void configureResetEncoders() {
  //   m_driverOI.getResetEncodersButton().whenPressed(new InstantCommand(m_drivetrain::resetEncoders, m_drivetrain));

  // }

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
    
  }
