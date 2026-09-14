package frc.robot.configuration;

import com.stzteam.mars.builder.Environment;
import com.stzteam.mars.builder.Environment.RunMode;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.operator.PS5OI;
import com.stzteam.mars.operator.XboxOI;

import frc.robot.configuration.constants.TunerConstants;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.SwerveTelemetry;

public class Manifest {

    public static final RunMode CURRENT_MODE = RunMode.SIM;

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

}
