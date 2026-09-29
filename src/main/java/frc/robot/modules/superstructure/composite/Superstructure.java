package frc.robot.modules.superstructure.composite;
/* 
import java.nio.channels.ClosedByInterruptException;

import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;
import com.stzteam.mars.models.multimodules.CompositeSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;

public class Superstructure extends CompositeSubsystem<SuperstructureData, SuperstructureIO>{

    public Superstructure(SuperstructureBuilder<SuperstructureData, SuperstructureIO> builder){
        super(builder);
    }

    public Dumper getDumper(){
        return getSubsystem(KeyManager.DUMPER_KEY);
    }

    //----------------------------------COMMANDS-------------------------------------------------------------

    public Command stopAll() {

    Dumper dumper = getDumper();
    //FlyWheel flywheel = getFlyWheelsIntake();
    //FlyWheel flywheelout = getFlywheelShooter();
    //Indexer index = getIndexer();
    //Intake intake = getIntake();

    return Commands.parallel(
        dumper.stop()
    );
    
 }

  @Override
  public SuperstructureData getState() {
    return inputs;
  }
}
*/