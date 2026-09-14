package frc.robot.modules;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class RobotLocalization {

    @FunctionalInterface
    public interface VisionConsumer<T, U, V> {
        void accept(T t, U u, V v);
    }

    public record MeasurementSample(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ){}

    public static record OdometryConfig(
        LinearAcceleration accThreshold,
        AngularVelocity maxAngularVelocity,
        LinearVelocity maxPlanarVelocity,
        double maxAmbiguityThreshold
    ){}

    public static record LocalizationInputs(
        Supplier<Rotation2d> gyroHeadingSupplier,
        Supplier<AngularVelocity> angularVelocitySupplier,
        Supplier<ChassisSpeeds> chassisSpeedsSupplier,
        Supplier<Vector<N2>> planarAccelerationVector
    ){}

    public static abstract class OdometrySource{

        private final String name;
        private Matrix<N3,N1> stdDevs;

        private OdometryConfig config;
        protected LocalizationInputs inputs;

        public OdometrySource(String name, Matrix<N3, N1> defaultStdDevs, OdometryConfig defaultConfig) {
            this.name = name;
            this.stdDevs = defaultStdDevs;
            this.config = defaultConfig;
        
        }

        void bindInputs(LocalizationInputs inputs) {
            this.inputs = inputs;
        }

        public Vector<N2> getPlanarVelocityVector() {
            if(inputs == null || inputs.chassisSpeedsSupplier() == null){
                return VecBuilder.fill(0.0, 0.0);
            }
            
            ChassisSpeeds speeds = inputs.chassisSpeedsSupplier().get();
            return VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }

        protected boolean isAngularVelocityValid() {
            if (inputs.angularVelocitySupplier() == null || config.maxAngularVelocity() == null) {
                return true;
            }
            double currentAngVel = Math.abs(inputs.angularVelocitySupplier().get().in(DegreesPerSecond));
            return currentAngVel <= config.maxAngularVelocity().in(DegreesPerSecond);
        }
        
        protected boolean isAccelerationValid() {
            if (inputs.planarAccelerationVector() == null || config.accThreshold() == null) {
                return true;
            }
            double currentAccelG = inputs.planarAccelerationVector().get().norm();
            return currentAccelG < config.accThreshold().in(Gs);
        }

        protected boolean isPlanarVelocityValid() {
            if (inputs.chassisSpeedsSupplier() == null || config.maxPlanarVelocity() == null) {
                return true;
            }

            double currentLinearVelMps = getPlanarVelocityVector().norm();
            return currentLinearVelMps <= config.maxPlanarVelocity().in(MetersPerSecond);
        }

        public final boolean isEnvironmentValid() {
            if (inputs == null || config == null) return true;

            return isAngularVelocityValid() 
            && isAccelerationValid() 
            && isPlanarVelocityValid();
        }

        public String getName() { return name; }
        public Matrix<N3, N1> getStdDevs() { return stdDevs; }
        public OdometryConfig getConfig() { return config; }

        public void setStdDevs(Matrix<N3, N1> newStdDevs) { this.stdDevs = newStdDevs; }
        public void setStdDevs(double stdX, double stdY, double stdTheta) { this.stdDevs = VecBuilder.fill(stdX, stdY, stdTheta); }
        public void setConfig(OdometryConfig newConfig) { this.config = newConfig; }

        public abstract boolean isPoseValid(Pose2d pose);
        public abstract Optional<MeasurementSample> getMeasurement();

    }

    public static final double INVALID_DEVIATION_ELEMENT = 999999.0;
    public static final Matrix<N3, N1> INVALID_DEVIATION = VecBuilder.fill(
        INVALID_DEVIATION_ELEMENT, INVALID_DEVIATION_ELEMENT, INVALID_DEVIATION_ELEMENT);

    private final VisionConsumer<Pose2d, Double, Matrix<N3, N1>> visionConsumer;
    private final LocalizationInputs inputs;
    private final Map<String, OdometrySource> sources = new HashMap<>();

    public RobotLocalization(
            VisionConsumer<Pose2d, Double, Matrix<N3, N1>> visionConsumer,
            LocalizationInputs inputs) {
        this.visionConsumer = visionConsumer;
        this.inputs = inputs;
    }

    public RobotLocalization registerSource(OdometrySource source) {
        source.bindInputs(inputs);
        sources.put(source.getName(), source);
        return this;
    }

    public Optional<OdometrySource> getSource(String name) {
        return Optional.ofNullable(sources.get(name));
    }

    public void update() {
        for (OdometrySource source : sources.values()) {
            if (!source.isEnvironmentValid()) continue;

            Optional<MeasurementSample> sampleOpt = source.getMeasurement();
            if (sampleOpt.isPresent()) {
                MeasurementSample sample = sampleOpt.get();
                visionConsumer.accept(sample.pose(), sample.timestampSeconds(), sample.stdDevs());
            }
        }
    }

}

