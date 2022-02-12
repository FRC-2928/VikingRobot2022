// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeSim extends LinearSystemSim<N1, N1, N1> {

    private boolean m_intakeSwitchActivatedSim = false;
    private boolean m_feederSwitchActivatedSim = false;
    private boolean m_intakeBrakeActivatedSim = false;
    private boolean m_feederBrakeActivatedSim = false;
    private boolean m_rampOpenSim = false;
    private int m_cycles = 0;

    public IntakeSim (LinearSystem<N1, N1, N1> plant)  {
        super(plant);
    }

    public IntakeSim ()  {
        super(
        LinearSystemId.identifyVelocitySystem(IntakeConstants.kvVoltSecondsPerMeter,
                                              IntakeConstants.kaVoltSecondsSquaredPerMeter));
    }

    // ------------- Intake --------------------------

    public void triggerActivateIntakeSwitchSim() {
        m_intakeSwitchActivatedSim = true;
        m_cycles = 0;
    }
    
      public void triggerDeactiveIntakeSwitchSim() {
        m_intakeSwitchActivatedSim = false;
    }

    // ------------- Feeder --------------------------
    public void triggerActivateFeederSwitchSim() {
        m_feederSwitchActivatedSim = true;
    }

    public void triggerDeactivateFeederSwitchSim() {
        m_feederSwitchActivatedSim = false;
    }

}
