package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;

public class DriverOI {
    private XboxController m_controller;

    private JoystickButton m_toggleIntakeMotor;
    private JoystickButton m_toggleFeederMotor;
    private JoystickButton m_shiftLow;
    private JoystickButton m_shiftHigh;
    private JoystickButton m_turnTurretToTarget;
    private JoystickButton m_incrementFlywheel;
    private JoystickButton m_decrementFlywheel;
    private JoystickButton m_toggleFlywheel;
    private Button m_turnTurretLeft;
    private Button m_turnTurretRight;

    public DriverOI(XboxController controller) {
        m_controller = controller;

        m_toggleIntakeMotor = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_toggleFeederMotor = new JoystickButton(m_controller, XboxController.Button.kB.value);
        m_shiftLow = new JoystickButton(m_controller, XboxController.Button.kX.value);
        m_shiftHigh = new JoystickButton(m_controller, XboxController.Button.kY.value);

        m_incrementFlywheel = new JoystickButton(m_controller, XboxController.Button.kStart.value);
        m_decrementFlywheel = new JoystickButton(m_controller, XboxController.Button.kBack.value);

        m_toggleFlywheel = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
        
        m_turnTurretToTarget = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
        
        m_turnTurretRight = new Button(() -> m_controller.getPOV() == 90);
        m_turnTurretLeft = new Button(() -> m_controller.getPOV() == 270);
    }

    // ---------------- Intake ----------------------------

    public Button getToggleIntakeMotorButton(){
        return m_toggleIntakeMotor;
    }

    public Button getToggleFeederMotorButton(){
        return m_toggleFeederMotor;
    }

    // ---------------- Climber ----------------------------


    // ---------------- Flywheel ----------------------------

    public Button getIncrementFlywheelButton(){
        return m_incrementFlywheel;
    }

    public Button getDecrementFlywheelButton(){
        return m_decrementFlywheel;
    }

    public Button getToggleFlywheelButton(){
        return m_toggleFlywheel;
    }

    // ---------------- Turret ---------------------------

    public DoubleSupplier getRotateTurretLeftSupplier() {
        return () -> m_controller.getLeftTriggerAxis();
    }

    public DoubleSupplier getRotateTurretRightSupplier() {
        return () -> m_controller.getRightTriggerAxis();
    }

    public Button getTurnTurretLeftButton(){
        return m_turnTurretLeft;
    }

    public Button getTurnTurretRightButton(){
        return m_turnTurretRight;
    }

    public Button getTurnTurretToTargetButton() {
        return m_turnTurretToTarget;
    }

    // ---------------- Drivetrain ----------------------------

    public Button getShiftLowButton() {
        return m_shiftLow;
    }


    public Button getShiftHighButton() {
        return m_shiftHigh;
    }

    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getLeftY();
    }


    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getRightX();
    }
}