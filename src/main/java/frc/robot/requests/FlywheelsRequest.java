package frc.robot.requests;

import com.stzteam.features.dictionary.Dictionary.CommonTables.Terminology;

import com.stzteam.features.dictionary.Dictionary.StatusCodes;
import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import frc.robot.configuration.constants.moduleconstants.flywheelsConstants;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIO;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheelsIO.FlyWheelsInputs;

@RequestFactory
public interface FlywheelsRequest extends Request<FlyWheelsInputs, flywheelsIO>{

    public static class IdelIntake implements FlywheelsRequest{

        @Override
        public ActionStatus apply(FlyWheelsInputs parameters, flywheelsIO actor) {
            actor.applyOutput(0);
            return ActionStatus.of(flywheels.IDLE, StatusCodes.IDLE_STATUS);
        }
    }

    public static class IdleOutake implements FlywheelsRequest {

        @Override
        public ActionStatus apply(FlyWheelsInputs parameters, flywheelsIO actor) {
            actor.applyOutput(flywheelsConstants.shooterWheelsConstants.idleVoltage);
            return ActionStatus.of(flywheels.IDLE, StatusCodes.IDLE_STATUS);
        }
    }
/* 
    @CreateCommand(name = "manual")
    public static class manualShoot implements FlywheelsRequest{
        private DoubleSupplier stick;

        public manualShoot getStick(DoubleSupplier stick){
            this.stick = stick;
            return this;
        }
        
        @Override
        public ActionStatus apply(FlyWheelInputs parameters, FlyWheelIO actor) {
            actor.setSpeed(stick.getAsDouble());
            return ActionStatus.of(FlyWheel.MANUAL_OVERRIDE, "Manual");
    }
    }
*/

    @CreateCommand(name = "dutyCycle")
    public static class moveSpeed implements FlywheelsRequest{
        double speed;

        public moveSpeed whiSpeed(double speed){
            this.speed = speed;
            return this;
        }

        @Override 
        public ActionStatus apply(FlyWheelsInputs parameters, flywheelsIO actor){
            actor.setSpeed(speed);
            return ActionStatus.of(flywheels.MANUAL_OVERRIDE, "Speed");
        }
    }

    @CreateCommand(name = "spinAtVoltage")
    public static class moveVoltage implements FlywheelsRequest {
        double volts;

        public moveVoltage whithVolts(Double target){
            this.volts = target;
            return this;
        }
        
        @Override
        public ActionStatus apply(FlyWheelsInputs parameters, flywheelsIO actor){
            actor.applyOutput(volts);
            return ActionStatus.of(flywheels.MANUAL_OVERRIDE, StatusCodes.MANUAL_STATUS + StatusCodes.voltsOf(volts));
        }    
    }    

    @CreateCommand(name = "toRPM")
    public static class SetRPM implements FlywheelsRequest{
        private double rpm;
        private double tolerance = 1.0;

        public SetRPM(double rpm){
            this.rpm = rpm;
        }

        public SetRPM toRPM(double rpm){
            this.rpm = rpm;
            return this;
        }

        public SetRPM whithToletance(double tol){
            this.tolerance = tol;
            return this;
        }

    @Override
    public ActionStatus apply(FlyWheelsInputs parameters, flywheelsIO actor) {
      parameters.targetRPM = rpm;
      actor.setTargetRPM(rpm);

      boolean isAtTarget = MathUtil.isNear(rpm, parameters.velocityRPM, tolerance);

      if (isAtTarget) {
        return ActionStatus.of(flywheels.ON_TARGET, StatusCodes.TARGETREACHED_STATUS);
      } else {
        return ActionStatus.of(
            flywheels.MOVING_TO_RPM, StatusCodes.TARGET_STATUS + rpm + Terminology.RPM);
      }
    }
    }
    
/* 
    @CreateCommand(name = "distanceToRPM")
  public static class InterpolateRPM implements FlywheelsRequest {
    private DoubleSupplier distanceMetersSupplier;
    private double toleranceRPM = 50.0;

    public InterpolateRPM withDistance(DoubleSupplier distanceSupplier) {
      this.distanceMetersSupplier = distanceSupplier;
      return this;
    }

    public InterpolateRPM withTolerance(double tol) {
      this.toleranceRPM = tol;
      return this;
    }

    @Override
    public ActionStatus apply(FlyWheelsInputs data, flywheelsIO actor) {

      double distance = distanceMetersSupplier.getAsDouble();

      double targetRPM = Constants.RPM_MAP.get(distance);

      data.targetRPM = targetRPM;
      actor.setTargetRPM(targetRPM);

      boolean isAtTarget = MathUtil.isNear(targetRPM, data.velocityRPM, toleranceRPM);

      if (isAtTarget) {
        return ActionStatus.of(flywheels.ON_TARGET, StatusCodes.TARGETREACHED_STATUS);
      } else {
        return ActionStatus.of(
            flywheels.MOVING_TO_RPM, StatusCodes.TARGET_STATUS + Math.round(targetRPM) + " RPM");
      }
    }
  }
*/

} 
