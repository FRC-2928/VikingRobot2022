package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;

public class DriverOI {
    private XboxController m_controller;

    // driver joystick
    private Joystick m_joystick = new Joystick(2);

    private Button m_toggleIntakeMotorButton;
    private Button m_toggleFeederMotorButton;

    public DriverOI(XboxController controller) {
        m_controller = controller;

        m_toggleIntakeMotorButton = new JoystickButton(m_joystick, 6);
        m_toggleFeederMotorButton = new JoystickButton(m_joystick, 7);
    }

    // ---------------- Intake ----------------------------


    // public Button getGroundIntakeButton() {
    //     return new Button(() -> m_controller.getLeftTriggerAxis() > 0.2);
    // }


    // public Button getStationIntakeButton() {
    //     return new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    // }

    public Button getToggleIntakeMotorButton(){
        // return new JoystickButton(m_controller, XboxController.Button.kA.value);
        return m_toggleIntakeMotorButton;
    }

    public Button getToggleFeederMotorButton(){
        // return new JoystickButton(m_controller, XboxController.Button.kB.value);
        return m_toggleFeederMotorButton;
    }

    // ---------------- Climber ----------------------------


    // public Button getClimbTrigger() {
    //     return new JoystickButton(m_controller, XboxController.Axis.kRightTrigger.value);
    // }

    // ---------------- Shooting ----------------------------


    
    // public Button getAutoShootingButton() {
    //     return new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    // }


    // public Button getSetpointShootingButton(){
    //     return new Button(() -> m_controller.getRightTriggerAxis() > 0.2);
    // }


    // public Button getFenderShotButton() {
    //     return new Button(() -> m_controller.getPOV() == 180);
    // }


    // public Button getInitiationlineShotButton() {
    //     return new Button(() -> m_controller.getPOV() == 0);
    // }


    // public Button getShooterDebugButton() {
    //     return new Button(() -> m_controller.getPOV() == 90);
    // }


    // public Button getFeedButton() {
    //     return new Button(() -> m_controller.getRightTriggerAxis() > 0.2);
    // }

    public Button getIncrementFlywheelButton(){
        return new JoystickButton(m_controller, XboxController.Button.kStart.value);
    }

    public Button getDecrementFlywheelButton(){
        return new JoystickButton(m_controller, XboxController.Button.kBack.value);
    }

    public Button getToggleFlywheelButton(){
        return new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    }

    public Button getTurnTurretLeftButton(){
        return new Button(() -> m_controller.getPOV() == 270);
    }

    public Button getTurnTurretRightButton(){
        return new Button(() -> m_controller.getPOV() == 90);
    }

    // ---------------- Drivetrain ----------------------------

    // public Button getResetEncodersButton() {
    //     return new JoystickButton(m_controller, XboxController.Button.kB.value);
    // }

    public Button getShiftLowButton() {
        return new JoystickButton(m_controller, XboxController.Button.kX.value);
    }


    public Button getShiftHighButton() {
        return new JoystickButton(m_controller, XboxController.Button.kY.value);
    }

    public Button getTurnTurretToTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    }


    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getLeftY();
    }


    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getRightX();
    }
}