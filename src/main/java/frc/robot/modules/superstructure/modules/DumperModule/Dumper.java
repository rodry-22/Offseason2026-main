package frc.robot.modules.superstructure.modules.DumperModule;

import java.util.function.Supplier;

import com.stzteam.features.dictionary.Dictionary.CommonTables;
import com.stzteam.features.dictionary.Dictionary.CommonTables.Terminology;
import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperInputs;
import frc.robot.requests.DumperCommands;
import frc.robot.requests.DumperRequest;
import frc.robot.requests.DumperRequestFactory;

public class Dumper extends ModularSubsystem<DumperInputs, DumperIO> implements DumperCommands{

    public static final ModuleColorCode IDLE =
      ModuleColorCode.solid("IDLE", Severity.OK, Color.kDarkGreen, "Dumper en reposo");
    public static final ModuleColorCode ON_TARGET =
      ModuleColorCode.solid("ON_TARGET", Severity.OK, Color.kFirstBlue, "Dumper en objetivo");
    public static final ModuleColorCode MOVING_TO_ANGLE =
      ModuleColorCode.solid(
          "MOVING_TO_ANGLE", Severity.WARNING, Color.kYellow, "Dumper moviéndose a %.2f grados");
    public static final ModuleColorCode MANUAL_OVERRIDE =
      ModuleColorCode.solid(
          "MANUAL_OVERRIDE", Severity.WARNING, Color.kPurple, "Dumper en control manual");
    public static final ModuleColorCode RESET =
      ModuleColorCode.solid("RESET", Severity.OK, Color.kDarkSalmon, "Dumper reiniciado");
    public static final ModuleColorCode OUT_OF_RANGE =
      ModuleColorCode.solid("OUT_OF_RANGE", Severity.ERROR, Color.kOrange, "Dumper fuera de rango");

    public Dumper(DumperIO io){
        super(
        SubsystemBuilder.<DumperInputs, DumperIO>setup()
            .key(KeyManager.DUMPER_KEY)
            .hardware(io, new DumperInputs())
            .request(DumperRequestFactory.idle())
            .telemetry(new DumperTelemetry()));

        this.setDefaultCommand(runRequest(() -> DumperRequestFactory.idle()));
      }

    public DumperInputs getState(){
        return inputs;
      }

    public boolean isAtTarget(double toleranceDegrees){
        return MathUtil.isNear(inputs.TargetAngle, inputs.position, toleranceDegrees);
      }

    @Override 
    public void absolutePeriodic(DumperInputs inputs){

      }


    public static class DumperTelemetry extends Telemetry<DumperInputs>{

        private static final String APPLIED_VOLTS_KEY = CommonTables.APPLIED_KEY + Terminology.VOLTS;

        @Override
        public void telemeterize(DumperInputs data) {

          
        NetworkIO.set(KeyManager.DUMPER_KEY, CommonTables.DEGREES_KEY, data.position);
        NetworkIO.set(KeyManager.DUMPER_KEY, CommonTables.TARGET_KEY, data.TargetAngle);
        NetworkIO.set(KeyManager.DUMPER_KEY, CommonTables.TIMESTAMP_KEY, data.timestamp);
        NetworkIO.set(KeyManager.DUMPER_KEY, APPLIED_VOLTS_KEY, data.appliedVolts);

        NetworkIO.set(KeyManager.DUMPER_KEY, "Current", data.current);
        }
        
      }

    @Override
    public Command setControl(Supplier<DumperRequest> request) {
      return setControl(request);    
    }


}

