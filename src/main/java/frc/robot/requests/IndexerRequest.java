package frc.robot.requests;

import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperInputs;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIO;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIO.IndexerInputs;


@RequestFactory
public interface IndexerRequest extends Request<IndexerInputs, IndexerIO> {
    
    public static ModuleColorCode IDLE =
        ModuleColorCode.solid("IDLE", Severity.OK, Color.kBlueViolet, "Indexer is idle");
    public static ModuleColorCode INDEXING_VOLTS=
        ModuleColorCode.solid("INDEXING_VOLTS", Severity.OK, Color.kSteelBlue, "Working index");
    public static ModuleColorCode INDEXING_RPM=
        ModuleColorCode.solid("INDEXING_RPM", Severity.OK, Color.kSteelBlue, "Working index");
    public static ModuleColorCode ROLLING_VOLTS=
        ModuleColorCode.solid("ROLLING_VOLTS", Severity.OK, Color.kAquamarine, "Working rollers");
    public static ModuleColorCode ROLLING_RPM=
        ModuleColorCode.solid("ROLLING_RPM", Severity.OK, Color.kAquamarine, "Working rollers");
    public static ModuleColorCode PROCESSING=
        ModuleColorCode.solid("PROCESSING", Severity.OK, Color.kAquamarine, "Rollers and Indexer working");

        @CreateCommand(name = "stop")
         public static class Idle implements IndexerRequest{
        @Override
        public ActionStatus apply(IndexerInputs data, IndexerIO actor) {
            actor.stopAll();
            return ActionStatus.of(IDLE, "Idle");
        }
    }

     @CreateCommand(name = "idleIndexer")
     public static class idleIndexer implements IndexerRequest{
        @Override
        public ActionStatus apply(IndexerInputs data, IndexerIO actor) {
            actor.stopAll();
            return ActionStatus.of(IDLE, "Indexer is idle");
        }
    }
    
    @CreateCommand(name = "setRollers")
    public static class setRollers implements IndexerRequest{
        public double m_volts = 0;
        public double RPM = 0;
        public setRollers withVolts(double volts){
            m_volts = volts;
            return this;
        }
        public setRollers withRPM(double RPM){
            this.RPM = RPM;
            return this;
        }
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            if(m_volts != 0){
                actor.applyRollers(m_volts);
                return ActionStatus.of(ROLLING_VOLTS, "Only rollers voltage");
            } else if(RPM != 0){
                actor.setRollers(RPM);
                return ActionStatus.of(ROLLING_RPM, "Only rollers RPM");
            } else {
                actor.stopRollers();
                return ActionStatus.of(IDLE, "Indexer is idle");
            }
        }
    }
    
    @CreateCommand(name = "setIndex")
    public static class setIndex implements IndexerRequest{
        public double m_volts = 0;
        public double RPM = 0;
        public setIndex withVolts(double volts){
            m_volts = volts;
            return this;
            }
        public setIndex withRPM(double RPM){
            this.RPM = RPM;
            return this;
            }
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            if(m_volts != 0){
                actor.applyIndex(m_volts);
                return ActionStatus.of(INDEXING_VOLTS, "Only index voltage");
            } else if(RPM != 0){
                actor.setIndex(RPM);
                return ActionStatus.of(INDEXING_RPM, "Only index RPM");
            } else {
                actor.stopIndex();
                return ActionStatus.of(IDLE, "Indexer is idle");
                }
            }
        }
        //  @CreateCommand(name = "processing")
        //  public static class Processing implements IndexerRequest{
        //     public double m_volts = 0;
        //     public double m_RPM = 0;
        //         public Processing withVolts(double volts){
        //             m_volts = volts;
        //             return this;
        //     }
        //         public Processing withRPM(double RPM){
        //             m_RPM = RPM;
        //             return this;
        //     }
        //     @Override
        //     public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
        //     if(m_volts != 0){
        //         actor.applyIndex(m_volts);
        //         return ActionStatus.of(PROCESSING, "Rollers and Index working");
        //     } else if(m_RPM != 0){
        //         actor.setIndex(m_RPM);
        //         return ActionStatus.of(PROCESSING, "Rollers and Index working");
        //     } else {
        //         actor.stopAll();
        //         return ActionStatus.of(IDLE, "Indexer is idle");
        //     }
        //     }
        // }
}
