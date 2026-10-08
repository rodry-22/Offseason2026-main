package frc.robot.modules.superstructure.modules.FlywheelsModule;

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
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIO.FlyWheelsInputs;
import frc.robot.requests.FlywheelsCommands;
import frc.robot.requests.FlywheelsRequest;
import frc.robot.requests.FlywheelsRequestFactory;

public class flywheels extends ModularSubsystem<FlyWheelsInputs, flywheelsIO> implements FlywheelsCommands{

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
            .key(key)
            .hardware(io, new FlyWheelsInputs())
            .request(FlywheelsRequestFactory.idleIntake())
            .telemetry(new flywheelsTelemetry(key))
        );

        this.mode = mode;
        this.subKey = key;


        if (mode == idleMode.intakeIDLE) {
            this.setDefaultCommand(runRequest(() -> FlywheelsRequestFactory.idleIntake()));
        } else {
            this.setDefaultCommand(runRequest(() -> FlywheelsRequestFactory.idleOutake()));
        }
    }


    public FlyWheelsInputs getState() {
        return inputs;
    }

    @Override
    public Command setControl(Supplier<FlywheelsRequest> request) {
    return runRequest(request);
  }

    public boolean isAtTarget(double toleranceRPM) {
    return MathUtil.isNear(inputs.targetRPM, inputs.velocityRPM, toleranceRPM);
  }

  public static class flywheelsTelemetry extends Telemetry<FlyWheelsInputs>{

    private static final String VELOCITY_RPM_KEY = CommonTables.VELOCITY_KEY + Terminology.RPM;
    private static final String APPLIED_VOLTS_KEY = CommonTables.APPLIED_KEY + Terminology.VOLTS;
    private static final String TARGET_RPM_KEY = CommonTables.TARGET_KEY + Terminology.RPM;

    String key;

    public flywheelsTelemetry(String key) {
    this.key = key;
    }

    @Override
    public void telemeterize(FlyWheelsInputs data){
        NetworkIO.set(key, VELOCITY_RPM_KEY, data.velocityRPM);
        NetworkIO.set(key, APPLIED_VOLTS_KEY, data.appliedVolts);
        NetworkIO.set(key, TARGET_RPM_KEY, data.targetRPM);

        NetworkIO.set(key, "Current", data.current);

        NetworkIO.set(key, "MotorRPM", data.motorRPM);
        NetworkIO.set(key, "FollowerRPM", data.followerRPM);
        NetworkIO.set(key, "FollowerCurrent", data.followerCurrent);
    }

  }

    @Override
    public void absolutePeriodic(FlyWheelsInputs inputs) {}

    @Override
    public void simulationPeriodic() {}
}