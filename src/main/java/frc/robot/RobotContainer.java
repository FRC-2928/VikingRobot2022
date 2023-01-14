package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transmission;
import frc.robot.subsystems.Turret;
import frc.robot.commands.ClimberCommands.ExtendClimberBars;
import frc.robot.commands.ClimberCommands.RetractClimberBars;
import frc.robot.commands.DrivetrainCommands.RunRamseteTrajectory;
import frc.robot.commands.FlywheelCommands.ToggleFlywheel;
import frc.robot.commands.IntakeCommands.CloseRamp;
import frc.robot.commands.IntakeCommands.EjectBall;
import frc.robot.commands.IntakeCommands.HoldIntakeUp;
import frc.robot.commands.IntakeCommands.OpenRamp;
import frc.robot.commands.IntakeCommands.ReverseFeeder;
import frc.robot.commands.IntakeCommands.ReverseFeederAndIntake;
import frc.robot.commands.IntakeCommands.ShootOnce;
import frc.robot.commands.IntakeCommands.ToggleFeederMotor;
import frc.robot.commands.IntakeCommands.ToggleIntakeMotor;
import frc.robot.commands.IntakeCommands.setLowIntakePower;
import frc.robot.commands.TurretCommands.MoveTurret;
import frc.robot.commands.TurretCommands.TurnTurretToTarget;

import frc.robot.commands.IntakeCommands.ShootAuto;

public class RobotContainer {

  // The Robot's Subsystems
  private final Transmission m_transmission = new Transmission();
  private final Drivetrain m_drivetrain = new Drivetrain(m_transmission::getGearState);
  private final Flywheel m_flywheel = new Flywheel();
  private final Turret m_turret = new Turret(m_drivetrain, m_flywheel);
  private final Intake m_intake = new Intake(m_drivetrain);
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
    new InstantCommand(m_climber::tiltBack);

    //sets the drivetrain to 90% for auto
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain.driveAuto(m_driverOI.getMoveSupplier(), m_driverOI.getRotateSupplier()),
            m_drivetrain));
  }

  public void onTeleopInit() {  
    new InstantCommand(m_climber::tiltBack);
    new InstantCommand(m_intake::allowIntakeUp, m_intake);

    //sets the drivetrain to 80% for teleop
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain.drive(m_driverOI.getMoveSupplier(), m_driverOI.getRotateSupplier()),
            m_drivetrain));
  }

  public void onRobotInit(){
    new InstantCommand(m_climber::tiltForward);
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

    m_driverOI.getShiftButton().whenPressed(new InstantCommand(m_transmission::toggle, m_transmission));

    m_autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup(new WaitCommand(0.1)));
    m_autoChooser.addOption("1-Ball Auto", new SequentialCommandGroup(new WaitCommand(1.0),
                                                                      new ShootAuto(m_intake, m_flywheel, m_turret), 
                                                                      new RunRamseteTrajectory(m_drivetrain, loadTrajectory("1BallAuto"))
                                                                      ));
    m_autoChooser.addOption("2-Ball Auto Right Curve", new SequentialCommandGroup(new ToggleIntakeMotor(m_intake), 
                                                                      new InstantCommand(m_intake::dontAllowIntakeUp, m_intake), 
                                                                      new InstantCommand(m_intake::setIntakeOut, m_intake), 
                                                                      new RunRamseteTrajectory(m_drivetrain, loadTrajectory("2BallP1")), 
                                                                      new RunRamseteTrajectory(m_drivetrain, loadTrajectory("2BallP2Red")), 
                                                                      new WaitCommand(1.0), 
                                                                      new ShootOnce(m_intake, m_flywheel, m_turret), 
                                                                      new WaitCommand(0.5),  
                                                                      new ShootOnce(m_intake, m_flywheel, m_turret),
                                                                      new InstantCommand(m_intake::allowIntakeUp, m_intake),
                                                                      new InstantCommand(m_intake::setIntakeUp, m_intake)
                                                                      ));
    m_autoChooser.addOption("2-Ball Auto Right Curve", new SequentialCommandGroup(new ToggleIntakeMotor(m_intake), 
                                                                      new InstantCommand(m_intake::dontAllowIntakeUp, m_intake), 
                                                                      new InstantCommand(m_intake::setIntakeOut, m_intake), 
                                                                      new RunRamseteTrajectory(m_drivetrain, loadTrajectory("2BallP1")), 
                                                                      new RunRamseteTrajectory(m_drivetrain, loadTrajectory("2BallP2Blue")), 
                                                                      new WaitCommand(1.0), 
                                                                      new ShootOnce(m_intake, m_flywheel, m_turret), 
                                                                      new WaitCommand(0.5),  
                                                                      new ShootOnce(m_intake, m_flywheel, m_turret),
                                                                      new InstantCommand(m_intake::allowIntakeUp, m_intake),
                                                                      new InstantCommand(m_intake::setIntakeUp, m_intake)
                                                                      ));
    
  }

  /**
   * Configure Turret
   */
  public void configureTurret() {
    
    // Configure default commands
    m_turret.setDefaultCommand(new TurnTurretToTarget(m_turret));
  
    // Configure button commands   
    m_operatorOI.getTrackTurretButton().whileHeld(new TurnTurretToTarget(m_turret));
    m_operatorOI.getTurnTurretLeftButton().whileHeld(new MoveTurret(m_turret, 1));
    m_operatorOI.getTurnTurretRightButton().whileHeld(new MoveTurret(m_turret, -1));
  }

  /**
   * Configure Intake
   */
  public void configureIntake() {
    // Configure default commands
    m_intake.setDefaultCommand(new RunCommand(m_intake::startMotors, m_intake));

    // Configure button commands

    m_driverOI.getToggleIntakeMotorButton().whenPressed(new ToggleIntakeMotor(m_intake));
    m_driverOI.getToggleFeederMotorButton().whenPressed(new ToggleFeederMotor(m_intake));
    m_driverOI.getIsAtHighSpeed().whileHeld(new setLowIntakePower(m_intake, m_drivetrain));
    m_driverOI.getIntakeOutButton().whileHeld(new HoldIntakeUp(m_intake));

    m_operatorOI.getCloseRamp().whenPressed(new CloseRamp(m_intake));
    m_operatorOI.getOpenRamp().whenPressed(new OpenRamp(m_intake));
    m_operatorOI.getShootBall().whenPressed(new ShootOnce(m_intake, m_flywheel, m_turret));
    m_operatorOI.getEjectBall().whenPressed(new EjectBall(m_intake));
    m_operatorOI.getReverseFeederButton().whenPressed(new ReverseFeeder(m_intake));
    m_operatorOI.getReverseIntakeButton().whenPressed(new ReverseFeederAndIntake(m_intake));
  }

  /**
   * Configure Flywheel
   */
  public void configureFlywheel() {

    // Configure default commands
    m_flywheel.setDefaultCommand(new RunCommand(m_flywheel::setVelocity, m_flywheel));
    
    // Configure button commands
    m_driverOI.getToggleFlywheelButton().whenPressed(new ToggleFlywheel(m_flywheel));
    m_driverOI.getIncrementFlywheelButton().whenPressed(new InstantCommand(m_flywheel::increaseFlywheelChange, m_flywheel));
    m_driverOI.getDecrementFlywheelButton().whenPressed(new InstantCommand(m_flywheel::decreaseFlywheelChange, m_flywheel));
    m_operatorOI.timeToClimb().whenPressed(new InstantCommand(m_flywheel::turnFlywheelOff, m_flywheel));
    m_operatorOI.getIncrementUpperFlywheelButton().whenPressed(new InstantCommand(m_flywheel::increaseUpperFlywheelChange));
    m_operatorOI.getDecrementUpperFlywheelButton().whenPressed(new InstantCommand(m_flywheel::decreaseUpperFlywheelChange));
    
  } 

  /**
   * Configure Climber
   */
  public void configureClimber() {

    // Configure default commands
    m_climber.setDefaultCommand(
        new RunCommand(() -> m_climber.moveClimber(m_operatorOI.getExtendRetractSupplier()), m_climber));

    // Configure button commands
    m_operatorOI.getExtendClimber().whileHeld(new ExtendClimberBars(m_climber));
    m_operatorOI.getRetractClimber().whileHeld(new RetractClimberBars(m_climber));
    m_operatorOI.getTiltForward().whenPressed(new InstantCommand(m_climber::tiltForward,m_climber));
    m_operatorOI.getTiltBack().whenPressed(new InstantCommand(m_climber::tiltBack,m_climber));
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

  public Climber getClimber(){
    return m_climber;
  }

  public boolean isClimberForward(){
    return m_climber.isClimberForward();
  }    
}