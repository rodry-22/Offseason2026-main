package frc.robot.modules.superstructure.modules.DumperModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Kilograms;



public class DumperIOSim implements DumperIO {

    private final SingleJointedArmSim simMotor;
    private final ProfiledPIDController simController;

    private double appliedVolts = 0.0;
    private Rotation2d currentTargetAngle = new Rotation2d();
    //FIXME: no me los sé
    private boolean isClosedLoop = false;
    private double gearRatio = 0;
    private final Distance radius = Meters.of(0);
    private  final Mass mass = Kilograms.of(0);
    private final double kLowerLimit = 0;
    private final double kUpperLimit = 0;
    private final boolean gravity = false;

    public DumperIOSim() {
        simMotor =
            new SingleJointedArmSim(
                DCMotor.getNEO(1),
                gearRatio,
                SingleJointedArmSim.estimateMOI(
                    radius.in(edu.wpi.first.units.Units.Meters),
                    mass.in(edu.wpi.first.units.Units.Kilograms)),
                radius.in(edu.wpi.first.units.Units.Meters),
                Units.degreesToRadians(kLowerLimit),
                Units.rotationsToRadians(kUpperLimit),
                gravity,
                0.0);

        simController = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(2.0, 4.0));
    }

    @Override
    public void updateInputs(DumperInputs inputs) {
        if (isClosedLoop) {
        // radianes (sim) a rotaciones (PID)
        double currentPosRotations = Units.radiansToRotations(simMotor.getAngleRads());
        appliedVolts = simController.calculate(currentPosRotations);
        }

        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
        simMotor.setInputVoltage(appliedVolts);
        simMotor.update(0.02);

        inputs.position = Units.radiansToDegrees(simMotor.getAngleRads());        
        inputs.VelocityRPS =Units.radiansPerSecondToRotationsPerMinute(simMotor.getVelocityRadPerSec()) / 60;
        inputs.TargetAngle = currentTargetAngle.getDegrees();
        inputs.appliedVolts = appliedVolts;
        inputs.Current = simMotor.getCurrentDrawAmps();
    }

    @Override
    public void setPosition(double angle, DumperMODE mode){
        isClosedLoop = true;
        this.currentTargetAngle = Rotation2d.fromDegrees(angle);

        switch (mode) {
            //FIXME: algo habrá que poner
            case kBACK: 
                simController.setPID(0, 0, 0);
                break;

            case kFRONT: 
                simController.setPID(0, 0, 0);
                break;
        }
        simController.setGoal(this.currentTargetAngle.getRotations());
    }

    @Override
    public void resetPosition(){
        simMotor.setState(0, 0.0);
    }
    @Override
    public void applyOutput(double volts){
        isClosedLoop = false;
        appliedVolts = volts;
    }
    @Override
    public void stopAll(){
        this.appliedVolts = 0;
    }
    
}
