package frc.robot.modules.superstructure.composite;

import com.stzteam.mars.models.multimodules.CompositeIO;

import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;

public class SuperstructureIO extends CompositeIO<SuperstructureData>{

    public SuperstructureIO (
        Dumper dumper,
        //Intake intake, 
        Indexer indexer
        //FlyWheel flywheelShooter,
        //FlyWheel flywheelsIntake
        ){

            registerChild(dumper);
            registerChild(indexer);

        }


    public SuperstructureIO(Dumper dumper, Indexer indexer, flywheels flywheels) {
        //TODO Auto-generated constructor stub
    }


    @Override 
    public void updateInputs(SuperstructureData inputs){}

    
}
