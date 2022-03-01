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

    public OperatorOI(XboxController controller) {
        m_controller = controller;

        // make sure these are built once
        m_printButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_extendClimber = new JoystickButton(m_controller, XboxController.Axis.kRightTrigger.value);
        m_retractClimber = new JoystickButton(m_controller, XboxController.Axis.kLeftTrigger.value);
        m_tiltForward = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
        m_tiltBack = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    }

    // ---------------- Intake ----------------------------


    // ---------------- Climber ----------------------------

    public Button getExtendClimber() {
        return m_extendClimber;
    }

    public Button getRetractClimber() {
        return m_retractClimber;
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

    public DoubleSupplier getRotateTurretSupplier() {
        return () -> m_controller.getRightX();
    }

    // ---------------- Drivetrain ----------------------------

}