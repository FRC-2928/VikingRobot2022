package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Transmission is responsible for shifting the gear on the drivetrain
 * Contains a statemachine for keeping gear state
 */

public class Transmission extends SubsystemBase {
  private Solenoid m_shiftPiston;
  private Solenoid m_shiftPistonHigh;
  private Solenoid m_shiftPistonLow;
  private GearState m_gearState;

  public enum GearState {
    HIGH, LOW;
  }

  public Transmission() {

    m_shiftPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.kDrivetrainShiftSolenoid);
    m_gearState = GearState.LOW;

  }

  public void setGearState(GearState state) {
    m_gearState = state;

    switch (state) {

    case HIGH:
      setFalse();
      break;

    case LOW:
      setTrue();
      break;
    }
  }

  public void setHigh(){
    setGearState(GearState.HIGH);
  }

  public void setLow(){
    setGearState(GearState.LOW);
  }

  public void toggle() {
    setGearState(m_gearState == GearState.LOW ? GearState.HIGH : GearState.LOW);
  }

  public GearState getGearState() {
    return m_gearState;
  }

  private void setTrue() {
    m_shiftPiston.set(true);
  }

  private void setFalse() {
    m_shiftPiston.set(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Gear State", m_gearState.toString());
  }
}
