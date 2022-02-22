// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.FlywheelConstants;

/** Add your docs here. */
public class TurretSim extends LinearSystemSim<N2, N1, N1> {

    public TurretSim(){
        super(LinearSystemId.identifyPositionSystem(FlywheelConstants.kvVoltSecondsPerMeter, 
                                                    FlywheelConstants.kaVoltSecondsSquaredPerMeter));
    }

    public TurretSim(LinearSystem<N2, N1, N1> linearSystem){
        super(linearSystem);
    }

    // public TurretSim(DCMotor motor, double radius, double mass, double gearRatio){
    //     super(LinearSystemId.createFlywheelSystem(motor, .5 * mass * radius * radius, gearRatio));
    // }

}
