// package frc.robot.modules.superstructure.modules.IndexerModule;

// import com.stzteam.mars.models.SubsystemBuilder;
// import com.stzteam.mars.models.singlemodule.ModularSubsystem;

// import frc.robot.configuration.KeyManager;
// import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIO.IndexerInputs;
// import frc.robot.requests.IndexerCommands;
// import frc.robot.requests.IndexerRequest;

// public class Indexer extends ModularSubsystem<IndexerInputs, IndexerIO> implements IndexerCommands{
//     public Indexer(IndexerIO io) {
//         super(
//             SubsystemBuilder.<IndexerInputs, IndexerIO>setup()
//                 .key(KeyManager.INDEXER_KEY)
//                 .hardware(io, new IndexerInputs())
//                 .request(new IndexerRequest.Idle())
//                 .telemetry(new IndexerTelemetry())
//         );
//         setDefaultCommand(runRequest(() -> new IndexerRequest.Idle()));
//     }