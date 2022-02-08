package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.oi.DriverOI;

public class DriverOI {
    private XboxController m_controller;

    public DriverOI(XboxController controller) {
        m_controller = controller;
    }

    // ---------------- Intake ----------------------------


    // public Button getGroundIntakeButton() {
    //     return new Button(() -> m_controller.getLeftTriggerAxis() > 0.2);
    // }


    // public Button getStationIntakeButton() {
    //     return new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    // }

    public Button getToggleIntakeMotorButton(){
        return new JoystickButton(m_controller, XboxController.Button.kA.value);
    }

    public Button getToggleFeederMotorButton(){
        return new JoystickButton(m_controller, XboxController.Button.kB.value);
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

    public Button getTurnToTargetButton() {
        return new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    }


    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getLeftY();
    }


    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getRightX();
    }
}