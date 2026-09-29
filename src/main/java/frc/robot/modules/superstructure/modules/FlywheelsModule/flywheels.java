package frc.robot.modules.superstructure.modules.FlywheelsModule;

import java.util.function.Supplier;

import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIO.FlyWheelsInputs;
import frc.robot.requests.FlywheelsRequest;

public class flywheels extends ModularSubsystem<FlyWheelsInputs, flywheelsIO> implements flywheelsCommands{

    public enum idleMode {
        intakeIDLE,
        outakeIDLE
    }

    public String subKey;

    public idleMode mode;

    
     public static final ModuleColorCode IDLE =
      ModuleColorCode.solid("IDLE", Severity.OK, Color.kDarkGreen, "Flywheel en reposo");
  public static final ModuleColorCode ON_TARGET =
      ModuleColorCode.solid("ON_TARGET", Severity.OK, Color.kFirstBlue, "En objetivo: %.2f RPM");
  public static final ModuleColorCode MOVING_TO_RPM =
      ModuleColorCode.solid(
          "MOVING_TO_RPM", Severity.WARNING, Color.kYellow, "Moviendo a %.2f RPM");
  public static final ModuleColorCode MANUAL_OVERRIDE =
      ModuleColorCode.solid(
          "MANUAL_OVERRIDE", Severity.WARNING, Color.kPurple, "Flywheel en control manual");
  public static final ModuleColorCode MANUAL_CONTROL =
      ModuleColorCode.solid(
          "MANUAL_CONTROL", Severity.WARNING, Color.kBrown, "Control manual: %.2fV");

    
    public flywheels(flywheelsIO io, String key, idleMode mode){
        super(
            SubsystemBuilder.<FlyWheelsInputs, flywheelsIO>setup()
            .key(KeyManager.FlYWHEELS_KEY)
            .hardware(io, new FlyWheelsInputs())
            .request(FlyWheelRequestFactory.IdelIntake())
            .telemetry(new flywheelsTelemetry(key))
        );

        this.mode = mode;

        if (mode == idleMode.intakeIDLE) {
            this.setDefaultCommand(runRequest(() -> FlyWheelRequestFactory.idleIntake()));
        } else {
            this.setDefaultCommand(runRequest(() -> FlyWheelRequestFactory.idleOutake()));
        }
    }

    @Override
    public FlyWheelsInputs getState() {
        return inputs;
    }

    @Override
    public Command setControl(Supplier<FlywheelsRequest> request) {
    return runRequest(request);
  }

    @Override
    public void absolutePeriodic(FlyWheelsInputs inputs) {}

    @Override
    public void simulationPeriodic() {}
}
