package frc.robot.modules.superstructure.composite;


import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.multimodules.CompositeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;

public class Superstructure extends CompositeSubsystem<SuperstructureData, SuperstructureIO>{

    public Superstructure(SubsystemBuilder<SuperstructureData, SuperstructureIO> builder){
        super(builder);
    }

    public Dumper getDumper(){
        return getSubsystem(KeyManager.DUMPER_KEY);
    }

    public Indexer getIndexer(){
        return getSubsystem(KeyManager.INDEXER_KEY);
    }

    //----------------------------------COMMANDS-------------------------------------------------------------

    public Command stopAll() {

    Dumper dumper = getDumper();
    //FlyWheel flywheel = getFlyWheelsIntake();
    //FlyWheel flywheelout = getFlywheelShooter();
    Indexer index = getIndexer();
    //Intake intake = getIntake();

    return Commands.parallel(
        dumper.sotop(),
        index.stop()
        
    );
    
 }
/*
  @Override
  public SuperstructureData getState() {
    return inputs;
  }
*/

@Override
  public void absolutePeriodic(SuperstructureData inputs) {
  }
}
