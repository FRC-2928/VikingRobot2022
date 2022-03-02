package frc.robot.oi;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class LogiTechDriverOI {
    // Buttons on the Logitch Gampad are numbered as integers counting from 1.
    // That means joystick button 0 does nothing.
    public static final int BUTTON_X     = 1;      // xBox = 3;
    public static final int BUTTON_A     = 2;      // xBox = 1;
    public static final int BUTTON_B     = 3;      // xBox = 2;
    public static final int BUTTON_Y     = 4;      // xBox = 4;
    public static final int BUTTON_LB    = 5;
    public static final int BUTTON_RB    = 6;
    public static final int BUTTON_LT    = 7;
    public static final int BUTTON_RT    = 8;
    public static final int BUTTON_BACK  = 9;
    public static final int BUTTON_START = 10;
    public static final int BUTTON_LEFT_STICK_PRESS  = 11;
    public static final int BUTTON_RIGHT_STICK_PRESS = 12;

    // Same buttons but mapped to number on the LogiTech Extreme
    public static final int BUTTON_1  = 2;
    public static final int BUTTON_2  = 2;
    public static final int BUTTON_3  = 3;      
    public static final int BUTTON_4  = 4;     
    public static final int BUTTON_5  = 5;
    public static final int BUTTON_6  = 6;
    public static final int BUTTON_7  = 7;
    public static final int BUTTON_8  = 8;
    public static final int BUTTON_9  = 9;
    public static final int BUTTON_10 = 10;
    public static final int BUTTON_11 = 11;
    public static final int BUTTON_12 = 12;

    // Axis of the gamepad's left and right joysticks.  Numbers 1 and 2 are for the 
    // left stick, 3 and 4 for the right stick.
    public static final int LEFT_X_AXIS  = 0;
    public static final int LEFT_Y_AXIS  = 1;
    public static final int RIGHT_X_AXIS = 2;      // Logitech gamepad = 3;
    public static final int RIGHT_Y_AXIS = 3;      // Logitech gamepad = 4;
    
    private Joystick m_controller;

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

    public LogiTechDriverOI(Joystick controller) {
        m_controller = controller;

        m_turnTurretToTarget = new JoystickButton(m_controller, BUTTON_1);
        // m_unassigned = new JoystickButton(m_controller, BUTTON_2);
        m_shiftLow = new JoystickButton(m_controller, BUTTON_3);
        m_shiftHigh = new JoystickButton(m_controller, BUTTON_4);  
        m_toggleIntakeMotor = new JoystickButton(m_controller, BUTTON_5);     
        m_toggleFeederMotor = new JoystickButton(m_controller, BUTTON_6);
        m_toggleFlywheel = new JoystickButton(m_controller, BUTTON_8);
        m_decrementFlywheel = new JoystickButton(m_controller, BUTTON_9);
        m_incrementFlywheel = new JoystickButton(m_controller, BUTTON_10);       
        m_turnTurretRight = new JoystickButton(m_controller, BUTTON_11);
        m_turnTurretLeft = new JoystickButton(m_controller, BUTTON_12);
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

    public Button getTurnTurretLeftButton(){
        return m_turnTurretLeft;
    }

    public Button getTurnTurretRightButton(){
        return m_turnTurretRight;
    }

    public Button getTurnTurretToTargetButton() {
        return m_turnTurretToTarget;
    }

    public DoubleSupplier getRotateTurretSupplier() {
        return () -> m_controller.getZ();
    }

    // ---------------- Drivetrain ----------------------------

    public Button getShiftLowButton() {
        return m_shiftLow;
    }


    public Button getShiftHighButton() {
        return m_shiftHigh;
    }

    public DoubleSupplier getMoveSupplier() {
        return () -> -m_controller.getX();
    }


    public DoubleSupplier getRotateSupplier() {
        return () -> m_controller.getY();
    }

}