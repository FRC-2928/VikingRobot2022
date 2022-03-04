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
    private Button m_tiltForward;
    private Button m_tiltBack;
    private JoystickButton m_printButton;
    private Button m_shootButton;
    private JoystickButton m_ejectButton;
    private Button m_turretTrackButton;
    private JoystickButton m_openRampButton;
    private JoystickButton m_closeRampButton;
    private JoystickButton m_turnTurretLeft;
    private JoystickButton m_turnTurretRight;
    private JoystickButton m_reverseFeederButton;

    public OperatorOI(XboxController controller) {
        m_controller = controller;


        // make sure these are built once
        m_extendClimber = new Button(() -> m_controller.getPOV() == 0);
        
        m_retractClimber =  new Button(() -> m_controller.getPOV() == 180);
        
        m_tiltForward = new Button(() -> m_controller.getPOV() == 90);
        m_tiltBack = new Button(() -> m_controller.getPOV() == 270);

        m_shootButton = new Button(() -> m_controller.getRightTriggerAxis() > 0.1);
        m_ejectButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
        m_openRampButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
        m_closeRampButton = new JoystickButton(m_controller, XboxController.Button.kB.value);

        m_reverseFeederButton = new JoystickButton(m_controller, XboxController.Button.kY.value);

        m_turnTurretRight = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
        m_turnTurretLeft = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);

        m_turretTrackButton = new Button(() -> m_controller.getLeftTriggerAxis() > 0.1);
        
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

    public Button getShootBall(){
        return m_shootButton;
    }

    public JoystickButton getReverseFeederButton(){
        return m_reverseFeederButton;
    }


    // ---------------- Climber ----------------------------

    public Button getExtendClimber() {
        return m_extendClimber;
    }

    public Button getRetractClimber() {
        return m_retractClimber;
    }

    public DoubleSupplier getExtendRetractSupplier() {
        return () -> m_controller.getLeftY();
    }

    // public DoubleSupplier getRetractSupplier() {
    //     return () -> m_controller.getRightTriggerAxis();
    // }

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

    public JoystickButton getTurnTurretLeftButton(){
        return m_turnTurretLeft;
    }

    public JoystickButton getTurnTurretRightButton(){
        return m_turnTurretRight;
    }

    public Button getTrackTurretButton(){
        return m_turretTrackButton;
    }

    // ---------------- Drivetrain ----------------------------

}