package frc.robot.requests;

import com.stzteam.features.dictionary.Dictionary.StatusCodes;
import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperInputs;
import frc.robot.modules.superstructure.modules.DumperModule.DumperIO.DumperMODE;
//import frc.robot.utils.LimelightHelpers;


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
/* 
        // Here -> get if it sees the hub and if it does, set the angle to the distance to the hub * 0.5 + 10

    @CreateCommand(name = "automaticAngle")
    public static class automaticAngle implements DumperRequest{
        @Override 
        public ActionStatus apply(DumperInputs parameters, DumperIO actor){
            LimelightHelpers.setPriorityTagID("", 4);
            LimelightHelpers.setPriorityTargetID("", 4);
            private DumperMODE mode = DumperMODE.kFRONT;

            double currentID = LimelightHelpers.getFiducialID("");
            boolean seesHub = LimelightHelpers.getTV("");

            double[] hubIDs = {1, 2, 3, 4, 5, 6, 7, 8};
            boolean isHub = false;
            for (double id : hubIDs) {
                if (currentID == id) {
                    isHub = true;
                    break; }
        }
        
           if (seesHub && isHub){
                double distanceToHub = LimelightHelpers.getTargetPose3d_CameraSpace("").getTranslation().getX();
                //parameters.distance_hub = distanceToHub;      
                double hub_hieght = 1.85; //meters
                double gravity = 9.81; //m/s^2
                double v0 = 0; //initial velocity
                double v0_2 = v0 * v0;
        
        
        double discrminant = Math.pow(v0, 4) - gravity * (gravity * Math.pow(distance, 2) + 2 * hub_hieght * v0_2);
`       if (discrminant < 0) {
            // No real solution, handle this case appropriately
            return;
        }

        double sqrtDiscriminant = Math.sqrt(discrminant);
        double angle1 = Math.atan((v0_2 + sqrtDiscriminant) / (gravity * distance));
        double angle2 = Math.atan((v0_2 - sqrtDiscriminant) / (gravity * distance));

        // Choose the appropriate angle based on the mode
        double chosenAngle;
        if (mode == DumperMODE.kBACK) {
            chosenAngle = Math.max(angle1, angle2); // Higher angle for back mode
        } else {
            chosenAngle = Math.min(angle1, angle2); // Lower angle for front mode
        }

        // Convert to degrees and set position
        position = Units.radiansToDegrees(chosenAngle);
        parameters.TargetAngle = position;
        
        actor.setPosition(position, mode);
        boolean isAtTarget = MathUtil.isNear(position, parameters.position, 1.0);

        if  (isAtTarget){
                return ActionStatus.of(Dumper.ON_TARGET, StatusCodes.TARGETREACHED_STATUS);
            } else {
                return ActionStatus.of(
                    Dumper.MOVING_TO_ANGLE, StatusCodes.TARGET_STATUS + StatusCodes.angleOf(position)
                );
            }
        }
    }
    }
    // here -> getpose 2d
    @CreateCommand(name = "Limiter")
    public static class Limiter implements DumperRequest{

        @Override 
        public ActionStatus apply(DumperInputs parameters, DumperIO actor){
            double targetAngle = parameters.distance_hub * 0.5 + 10;
            parameters.TargetAngle = targetAngle;
            actor.setPosition(targetAngle, DumperMODE.kFRONT);

            boolean isAtTarget = MathUtil.isNear(targetAngle, parameters.position, 1.0);

            if  (isAtTarget){
                return ActionStatus.of(Dumper.ON_TARGET, StatusCodes.TARGETREACHED_STATUS);
            } else {
                return ActionStatus.of(
                    Dumper.MOVING_TO_ANGLE, StatusCodes.TARGET_STATUS + StatusCodes.angleOf(targetAngle)
                );
            }
        }
    }   
    */

    @CreateCommand(name = "setAngle")
    public static class setAngle implements DumperRequest{

        private double angle;
        private double tolerance = 1.0;
        private DumperMODE mode = DumperMODE.kFRONT;

        public setAngle(double initialAngle){
            this.angle = initialAngle;
        } 

        public setAngle withAngle(double angle){
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

        @Override 
        public ActionStatus apply(DumperInputs parameters, DumperIO actor){
        actor.applyOutput(voltage);
        return ActionStatus.of(
            Dumper.MANUAL_OVERRIDE, StatusCodes.MANUAL_STATUS + StatusCodes.voltsOf(voltage)
        );
    }
    }


    
}

