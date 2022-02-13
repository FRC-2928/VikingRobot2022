// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.FlywheelConstants;

/** Add your docs here. */
public class FlywheelSim extends LinearSystemSim<N1, N1, N1> {

    public FlywheelSim(){
        super(LinearSystemId.identifyVelocitySystem(FlywheelConstants.kvVoltSecondsPerMeter, 
                                                    FlywheelConstants.kaVoltSecondsSquaredPerMeter));
    }

    public FlywheelSim(LinearSystem<N1, N1, N1> linearSystem){
        super(linearSystem);
    }

    public FlywheelSim(DCMotor motor, double radius, double mass, double gearRatio){
        //moment of inertia is .5m*r^2 ...?
        super(LinearSystemId.createFlywheelSystem(motor, .5 * mass * radius * radius, gearRatio));
    }

}
