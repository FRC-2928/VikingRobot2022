// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.Limelights;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final Limelight m_turretLimelight;
  private final TalonSRX m_turretMotor;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------
  public Turret() {
    m_turretLimelight = new Limelight(Limelights.TURRET);
    m_turretMotor = new TalonSRX(Constants.RobotMap.kTurretSparkMax);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------

}
