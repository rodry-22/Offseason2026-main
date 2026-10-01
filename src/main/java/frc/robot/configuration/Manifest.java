package frc.robot.configuration;

import com.stzteam.mars.builder.Environment;
import com.stzteam.mars.builder.Injector;
import com.stzteam.mars.builder.Environment.RunMode;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.operator.PS5OI;
import com.stzteam.mars.operator.XboxOI;

import frc.robot.configuration.constants.TunerConstants;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIO;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIOFallback;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIOSim;
import frc.robot.modules.superstructure.modules.IndexerModule.IndexerIOSpark;
import frc.robot.modules.superstructure.modules.IntakeModule.Intake;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeIO;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeIOFallback;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeIOSim;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeSpark;
import frc.robot.modules.superstructure.composite.Superstructure;
import frc.robot.modules.superstructure.composite.SuperstructureData;
import frc.robot.modules.superstructure.composite.SuperstructureIO;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIOFallback;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIOSim;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIOSpark;
import frc.robot.modules.superstructure.modules.FlywheelsModule.FlyWheelIOKrakenIntake;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIO;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIOFallback;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIOSim;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIOkrakenShooter;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels.idleMode;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.SwerveTelemetry;

public class Manifest {

    public static final RunMode CURRENT_MODE = RunMode.REAL;

    public enum ControllerType {
        PS5,
        XBOX
    }

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;
    
        static{Environment.setMode(CURRENT_MODE);}
    
        public static final ControllerType DRIVER_CONTROLLER = ControllerType.XBOX;
        public static final ControllerType OPERATOR_CONTROLLER = ControllerType.XBOX;
    
        public static final boolean HAS_DRIVETRAIN = true;
        public static final boolean HAS_INDEXER = true;
        public static final boolean HAS_DUMPER = true;
        public static final boolean HAS_SHOOTER_WHELLS = true;
        public static final boolean HAS_INTAKE_FLYWHELLS = true;
        public static final boolean HAS_INTAKE_WHEELS = true;
    
    
    
    
        public static class SuperstructureBuilder {
        public static Superstructure superBuild(
            Dumper dumper,  
            Intake intake,
            Indexer indexer,
            flywheels flywheelsShooter,
            flywheels flywheelsIntake
            ){
    
          SuperstructureIO io =
              new SuperstructureIO(dumper, intake, indexer, flywheelsShooter,flywheelsIntake);
    
            return new Superstructure(SubsystemBuilder.<SuperstructureData, SuperstructureIO>setup()
            .key(KeyManager.SUPERSTRUCTURE_KEY).hardware(io, new SuperstructureData()));
        }
      }
    
        public static class ControlsBuilder {
    
        public static ControllerOI buildDriver() {
          return DRIVER_CONTROLLER == ControllerType.PS5
              ? new PS5OI(DRIVER_PORT)
              : new XboxOI(DRIVER_PORT);
        }
    
        public static ControllerOI buildOperator() {
          return OPERATOR_CONTROLLER == ControllerType.PS5
              ? new PS5OI(OPERATOR_PORT)
              : new XboxOI(OPERATOR_PORT);
        }
        }
    
        public static class DrivetrainBuilder {
    
        public static CommandSwerveDrivetrain buildModule() {
            if (!HAS_DRIVETRAIN) return null;
    
            CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
            SwerveTelemetry telemetry = new SwerveTelemetry();
            drivetrain.registerTelemetry(telemetry::telemeterize);
    
            return drivetrain;
            }
        }
        public static Indexer buildIndexer() {
        IndexerIO io =
            Injector.createIO(HAS_INDEXER, IndexerIOFallback::new, IndexerIOSpark::new, IndexerIOSim::new);
            return new Indexer(io);
      }

      public static Intake buildIntake(){
        IntakeIO io = Injector.createIO(HAS_INTAKE_WHEELS, IntakeIOFallback::new, IntakeSpark::new, IntakeIOSim::new);
        return new Intake(io);
      }
    
        public static Dumper buildDumper(){
            DumperIO io = Injector.createIO(HAS_DUMPER, DumperIOFallback::new, DumperIOSpark::new, DumperIOSim::new);
            return new Dumper(io);
    
        }
    
        public static flywheels buildFlywheelsShooter(){
          flywheelsIO io = Injector.createIO(HAS_SHOOTER_WHELLS, flywheelsIOFallback::new, flywheelsIOkrakenShooter::new, flywheelsIOSim::new);
          return new flywheels(io, KeyManager.FLYWHEELS_SHOOTER_KEY, idleMode.outakeIDLE);
        }
    
        public static flywheels buildFlywheelsIntake() {
        flywheelsIO io =
            Injector.createIO(HAS_INTAKE_FLYWHELLS, flywheelsIOFallback::new, FlyWheelIOKrakenIntake::new, flywheelsIOSim::new);

    return new flywheels(io, KeyManager.FLYWHEELS_INTAKE_KEY, idleMode.intakeIDLE);
  }


}


