package frc.robot.requests;
/* 
import java.io.ObjectInputFilter.Status;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.StatusCode;
import com.stzteam.features.dictionary.Dictionary.StatusCodes;
import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperInputs;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperMODE;


@RequestFactory
public interface DumperRequest extends Request<DumperInputs, DumperIO>{

    @CreateCommand(name = "sotop")
    public static class Idle implements DumperRequest{
        @Override
        public ActionStatus apply(DumperInputs data, DumperIO actor) {
            actor.stopAll();
            return ActionStatus.of(Dumper.IDLE, "Idle");
        }
    }

    @CreateCommand(name = "speed")
    public static class resetPosition implements DumperRequest {
        @Override 
        public ActionStatus apply(DumperInputs data,DumperIO actor){
            actor.resetPosition();
                return ActionStatus.of(Dumper.RESET, "Reseted");
        }
    }

    @CreateCommand(name = "setAngle")
    public static class setAngle implements DumperRequest{

        private double angle;
        private double tolerance = 1.0;
        private DumperMODE mode = DumperMODE.kFRONT;

        public setAngle(double initialAngle){
            this.angle = initialAngle;
        } 

        public setAngle whhAngle(double angle){
            this.angle = angle;
            return this;
        }

        public setAngle withMode(DumperMODE mode){
            this.mode = mode;
            return this;
        }

        public setAngle Tolerance(double tolerance){
            this.tolerance = tolerance;
            return this;
        }

        @Override 
        public ActionStatus apply(DumperInputs parameters, DumperIO actor){
            parameters.TargetAngle = angle;
            actor.setPosition(angle, mode);

            boolean isAtTarget = MathUtil.isNear(angle, parameters.position, tolerance);

            if  (isAtTarget){
                return ActionStatus.of(Dumper.ON_TARGET, StatusCodes.TARGETREACHED_STATUS);
            } else {
                return ActionStatus.of(
                    Dumper.MOVING_TO_ANGLE, StatusCodes.TARGET_STATUS + StatusCodes.angleOf(angle)
                );
            }
        }
    }

   @CreateCommand (name = "voltageCommand")
    public static class moveVoltage implements DumperRequest{
        public double voltage;

        public moveVoltage withvolVolts(double volts){
            this.voltage = volts;
            return this;
        }
    }

    @Override 
    public ActionStatus apply(DumperInputs parameters, DumperIO actor){
        actor.applyOutput(voltage);
        return ActionStatus.of(
            Dumper.MANUAL_OVERRIDE, StatusCodes.MANUAL_STATUS + StatusCodes.voltsOf(voltage)
        );
    }
    
}
    */
