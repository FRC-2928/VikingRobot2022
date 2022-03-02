package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;

public class OperatorOI {
    private XboxController m_controller;

    // buttons
    private JoystickButton m_extendClimber;
    private JoystickButton m_retractClimber;
    private JoystickButton m_tiltForward;
    private JoystickButton m_tiltBack;
    private JoystickButton m_printButton;
    private Button m_turnTurretLeft;
    private Button m_turnTurretRight;

    public OperatorOI(XboxController controller) {
        m_controller = controller;

        // make sure these are built once
        m_extendClimber = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_retractClimber = new JoystickButton(m_controller, XboxController.Button.kB.value);
        m_tiltForward = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
        m_tiltBack = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);

        m_turnTurretRight = new Button(() -> m_controller.getPOV() == 90);
        m_turnTurretLeft = new Button(() -> m_controller.getPOV() == 270);
    }

    // ---------------- Intake ----------------------------


    // ---------------- Climber ----------------------------

    public JoystickButton getExtendClimber() {
        return m_extendClimber;
    }

    public JoystickButton getRetractClimber() {
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

    // ---------------- Drivetrain ----------------------------

}