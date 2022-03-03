package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;

public class OperatorOI {
    private XboxController m_controller;

    // buttons
    private Button m_extendClimber;
    private Button m_retractClimber;
    private JoystickButton m_tiltForward;
    private JoystickButton m_tiltBack;
    private JoystickButton m_printButton;
    private JoystickButton m_shootButton;
    private JoystickButton m_ejectButton;
    private JoystickButton m_turretTrackButton;
    private JoystickButton m_openRampButton;
    private JoystickButton m_closeRampButton;
    private Button m_turnTurretLeft;
    private Button m_turnTurretRight;

    public OperatorOI(XboxController controller) {
        m_controller = controller;


        // make sure these are built once
        m_extendClimber = new Button(() -> m_controller.getRightTriggerAxis() > 0);
        m_retractClimber = new Button(() -> m_controller.getLeftTriggerAxis() > 0);
        m_tiltForward = new JoystickButton(m_controller, XboxController.Button.kStart.value);
        m_tiltBack = new JoystickButton(m_controller, XboxController.Button.kBack.value);

        m_shootButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
        m_ejectButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
        m_openRampButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_closeRampButton = new JoystickButton(m_controller, XboxController.Button.kB.value);

        m_turnTurretRight = new Button(() -> m_controller.getPOV() == 90);
        m_turnTurretLeft = new Button(() -> m_controller.getPOV() == 270);
    }

    // ---------------- Intake ----------------------------

    public JoystickButton getOpenRamp(){
        return m_openRampButton;
    }

    public JoystickButton getCloseRamp(){
        return m_closeRampButton;
    }

    public JoystickButton getEjectBall(){
        return m_ejectButton;
    }

    public JoystickButton getShootBall(){
        return m_shootButton;
    }

    // ---------------- Climber ----------------------------

    public Button getExtendClimber() {
        return m_extendClimber;
    }

    public Button getRetractClimber() {
        return m_retractClimber;
    }

    public DoubleSupplier getExtendSupplier() {
        return () -> m_controller.getLeftTriggerAxis();
    }

    public DoubleSupplier getRetractSupplier() {
        return () -> m_controller.getRightTriggerAxis();
    }

    public Button getTiltForward() {
        return m_tiltForward;
    }

    public Button getTiltBack() {
        return m_tiltBack;
    }
    // ---------------- Flywheel ----------------------------
 
    public Button getPrintButton() {
      return m_printButton;
    }
 
    // ---------------- Turret ----------------------------

    public Button getTurnTurretLeftButton(){
        return m_turnTurretLeft;
    }

    public Button getTurnTurretRightButton(){
        return m_turnTurretRight;
    }

    public JoystickButton getTrackTurretButton(){
        return m_turretTrackButton;
    }

    // ---------------- Drivetrain ----------------------------

}