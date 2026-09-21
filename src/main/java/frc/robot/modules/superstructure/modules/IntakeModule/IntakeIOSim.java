package frc.robot.modules.superstructure.modules.IntakeModule;
import frc.robot.configuration.constants.moduleconstants.IntakeConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {

    public double moiRolls; 
    public double moiAngulator; 
    private double appliedVolts;
    private final DCMotor gearbox;
    private final double gearing;
    private final SingleJointedArmSim armSim;
    private final FlywheelSim simMotor;

    public IntakeIOSim() {

        this.moiRolls = IntakeConstants.MOIRolls; // I =1/3 mL^2
        this.moiAngulator = IntakeConstants.MOIAngulator; // I =1/3 mL^2
        appliedVolts = 0.0;
        this.gearbox = DCMotor.getNEO(IntakeConstants.numMotors); //cambiar # motores usados
        this.gearing = IntakeConstants.simGearing;
        this.armSim = new SingleJointedArmSim(gearbox, 
            gearing, 
            moiAngulator, 
            IntakeConstants.armLength, 
            IntakeConstants.minAngle, 
            IntakeConstants.maxAngle, 
            false, 
            0.0);

        FlywheelSim simMotor = new FlywheelSim(null, gearbox, null); 
        
    }   

    
}
