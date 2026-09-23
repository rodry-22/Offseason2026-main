package frc.robot.modules.superstructure.modules.IndexerModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.configuration.constants.moduleconstants.IndexerConstants;

public class IndexerIOSim implements IndexerIO{

    private final FlywheelSim simIndex;
    private final FlywheelSim simRoller;
    
    private double indexVolts = 0;
    private double rollerVolts = 0;
    private double indexRPS = 0; //not sure if i should use simIndexRPS
    private double rollerRPS = 0;

    public IndexerIOSim() {

        var indexGearbox = DCMotor.getNEO(1);
        var indexPlant = LinearSystemId.createFlywheelSystem(indexGearbox, IndexerConstants.kIndexMOI, IndexerConstants.indexRatio);
        simIndex = new FlywheelSim(
                    indexPlant, 
                    indexGearbox, 
                    IndexerConstants.indexRatio);
                    
        var rollerGearbox = DCMotor.getNEO(1);
        var rollerPlant = LinearSystemId.createFlywheelSystem(rollerGearbox, IndexerConstants.kRollerMOI, IndexerConstants.rollerRatio);
        simRoller = new FlywheelSim(
                    rollerPlant, 
                    rollerGearbox, 
                    IndexerConstants.rollerRatio);
                       
    }
     @Override
    public void updateInputs(IndexerInputs inputs) {
        indexVolts = MathUtil.clamp(simIndex.getInputVoltage(), -12, 12);
        rollerVolts = MathUtil.clamp(simRoller.getInputVoltage(), -12, 12);
        
        simIndex.update(0.02);
        simRoller.update(0.02);
        
        simIndex.setInputVoltage(indexVolts);
        simRoller.setInputVoltage(rollerVolts);
        
        inputs.indexRPS = simIndex.getAngularVelocityRPM() / 60.0;
        inputs.rollerRPS = simRoller.getAngularVelocityRPM() / 60.0;

        simIndex.setAngularVelocity(
            Units.rotationsPerMinuteToRadiansPerSecond(indexRPS*60)); //in radians/second}

        simRoller.setAngularVelocity(
            Units.radiansPerSecondToRotationsPerMinute(rollerRPS*60)); //in radians/second

    }
    @Override
    public void applyRollers(double volts) {
        rollerVolts = volts;
    }
    @Override
    public void applyIndex(double volts) {
        indexVolts = volts;
    }
    @Override
    public void setRollers(double RPS) {
        rollerRPS = RPS;
        
    }
    @Override
    public void setIndex(double RPS) {
        indexRPS = RPS;
    }
    @Override
    public void stopRollers() {
        rollerRPS = 0;
    }
    @Override
    public void stopIndex() {
        indexRPS = 0;
    }
    @Override
    public void stopAll() {
        stopRollers();
        stopIndex();
    }

}
