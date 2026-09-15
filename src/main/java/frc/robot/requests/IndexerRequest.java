package frc.robot.requests;

import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIO;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIO.IndexerInputs;

@RequestFactory
public interface IndexerRequest extends Request<IndexerInputs, IndexerIO> {
    
    public static ModuleColorCode IDLE =
        ModuleColorCode.solid("IDLE", Severity.OK, Color.kBlueViolet, "Index is idle");
    public static ModuleColorCode INDEXING_VOLTS=
        ModuleColorCode.solid("INDEXING_VOLTS", Severity.OK, Color.kSteelBlue, "Working index"); //TODO: find a good name. Subsytem is called the same and it's confusing
    public static ModuleColorCode INDEXING_RPS=
        ModuleColorCode.solid("INDEXING_RPS", Severity.OK, Color.kSteelBlue, "Working index"); //TODO: find a good name. Subsytem is called the same and it's confusing
    public static ModuleColorCode ROLLING_VOLTS=
        ModuleColorCode.solid("ROLLING_VOLTS", Severity.OK, Color.kAquamarine, "Working rollers");
    public static ModuleColorCode ROLLING_RPS=
        ModuleColorCode.solid("ROLLING_RPS", Severity.OK, Color.kAquamarine, "Working rollers");
    public static ModuleColorCode PROCESSING=
        ModuleColorCode.solid("PROCESSING", Severity.OK, Color.kAquamarine, "Rollers and Indexer working");
    //public static ModuleColorCode 

     @CreateCommand(name = "idle")
     public static class Idle implements IndexerRequest{
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            actor.stopAll();
            return ActionStatus.of(IDLE, "Indexer is idle");
        }
    }
     @CreateCommand(name = "processing")
     public static class Processing implements IndexerRequest{
        public double m_volts = 0; 
            public Processing withVolts(double volts){
                m_volts = volts;
                return this;
        }
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            actor.applyIndexer(m_volts);
            actor.applyRollers(m_volts);
            return ActionStatus.of(PROCESSING, "Rollers and Index working");
        }
    }


    /*
     * I've got some questions with the _Volts commands. Not sure if there should be a default Voltage
     */
    @CreateCommand(name = "rollersVolts")
    public static class Rollers implements IndexerRequest{
        public double m_volts = 0; //TODO: find a good default value?
        public Rollers withVolts(double volts){
            m_volts = volts;
            return this;
        }
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            actor.applyRollers(m_volts);
            return ActionStatus.of(ROLLING_VOLTS, "Only rollers working");
        }
    }

    @CreateCommand(name = "indexVolts")
    public static class Index implements IndexerRequest{
        public double m_volts = 0;
        public Index withVolts(double volts){
            m_volts = volts;
            return this;
        }
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            actor.applyIndexer(m_volts);
            return ActionStatus.of(INDEXING_VOLTS, "Only index working");
        }
    }
    //Adding a command to individually stop the indexer and rollers, could be useful
    //TODO: Use the RPS commands
}
