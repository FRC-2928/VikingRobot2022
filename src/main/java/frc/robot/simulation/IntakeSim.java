// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeSim extends LinearSystemSim<N1, N1, N1> {

    private boolean m_intakeSwitchClosed = false;
    private boolean m_feederSwitchClosed = false;
    private boolean m_rampOpenSim = false;
    private Alliance m_ballColor = Alliance.Invalid;

    public IntakeSim (LinearSystem<N1, N1, N1> plant)  {
        super(plant);
    }

    public IntakeSim ()  {
        super(
        LinearSystemId.identifyVelocitySystem(IntakeConstants.kvVoltSecondsPerMeter,
                                              IntakeConstants.kaVoltSecondsSquaredPerMeter));
    }

    // ------------- Intake --------------------------

    public void triggerCloseIntakeSwitchSim() {
        m_intakeSwitchClosed = true;
    }
    
      public void triggerOpenIntakeSwitchSim() {
        m_intakeSwitchClosed = false;
    }

    public boolean isIntakeSwitchClosed() {
        return m_intakeSwitchClosed;
    }

    // ------------- Feeder --------------------------
    public void triggerCloseFeederSwitchSim() {
        m_feederSwitchClosed = true;
    }

    public void triggerOpenFeederSwitchSim() {
        m_feederSwitchClosed = false;
    }

    public boolean isFeederSwitchClosed() {
        return m_feederSwitchClosed;
    }

    // ------------- Ball --------------------------
    public Alliance getBallColor() {
        m_ballColor = Alliance.Blue;
        return m_ballColor;
    }

    public void setBallColor(Alliance ballColor) {
        m_ballColor = ballColor;
    }

}
