package frc.robot.modules.superstructure.composite;

import com.stzteam.mars.models.multimodules.CompositeIO;

import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;

public class SuperstructureIO extends CompositeIO<SuperstructureData>{

    public SuperstructureIO (
        Dumper dumper,
        //Intake intake, 
        Indexer indexer,
        flywheels flywheelShooter,
        flywheels flywheelsIntake){

        registerChild(dumper);
        registerChild(indexer);
        registerChild(flywheelsIntake);
        registerChild(flywheelShooter);

        }



    


    @Override 
    public void updateInputs(SuperstructureData inputs){}

    
}
