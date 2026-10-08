// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.IdealStartingState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.DriveFeedforwards;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.configuration.constants.moduleconstants.SwerveConstants;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.Set;
// import java.util.function.Consumer;
// import java.util.function.Supplier;

// /**
//  * Autotune del PID de seguimiento de trayectoria (PPHolonomicDriveController) sobre el robot real
//  * (o en sim). Corre paths de ida y vuelta desde la pose donde esta el robot, mide el error entre la
//  * pose objetivo de PathPlanner y la pose real, y se queda con las ganancias de menor costo.
//  *
//  * <p>Se usa desde SmartDashboard/Elastic, tabla "PathTuning". El robot se mueve solo: deja libre
//  * ~{@code Distance} metros al frente del robot y ten a alguien listo para deshabilitar.
//  */
// public final class PathFollowingTuner {

//   private static final String TABLE = "PathTuning/";

//   /** Si el error de posicion pasa de esto, se aborta el tramo (ganancias inestables). */
//   private static final double ABORT_ERROR_METERS = 0.5;

//   private static final double ABORTED_COST = 10.0;

//   /**
//    * Giro de las etapas de rotacion. No 180: girar media vuelta en 2.5 m a 3 m/s pide ~5.4 m/s en
//    * las ruedas de afuera, casi la velocidad libre del Kraken sin FOC, y el robot no llega por
//    * torque (medido en Gazebo: 27 cm de error final con cualquier ganancia). Eso no compara
//    * ganancias, compara contra una trayectoria imposible.
//    */
//   private static final double ROTATION_TEST_DEG = 90.0;
//   private static final double SETTLE_SECONDS = 0.4;

//   private static final double[] TRANSLATION_KP_CANDIDATES = {2.0, 3.5, 5.0, 6.5, 8.0, 10.0, 12.0};
//   private static final double[] ROTATION_KP_CANDIDATES = {2.0, 3.5, 5.0, 6.5, 8.0, 10.0};
//   private static final double[] KD_CANDIDATES = {0.0, 0.05, 0.1, 0.2, 0.4};

//   private static final int KP_TRANS = 0;
//   private static final int KD_TRANS = 1;
//   private static final int KP_ROT = 2;
//   private static final int KD_ROT = 3;

//   private final CommandSwerveDrivetrain drivetrain;
//   private final TrackingStats stats = new TrackingStats();

//   /** Ganancias de trabajo del autotune: [kPTrans, kDTrans, kPRot, kDRot]. */
//   private final double[] bestGains = new double[4];

//   private double bestStageCost;
//   private double candidateValue;
//   private Pose2d anchor;

//   public PathFollowingTuner(CommandSwerveDrivetrain drivetrain) {
//     this.drivetrain = drivetrain;

//     SmartDashboard.putNumber(TABLE + "kP_Trans", SwerveConstants.PathTranslationPID.kP);
//     SmartDashboard.putNumber(TABLE + "kD_Trans", SwerveConstants.PathTranslationPID.kD);
//     SmartDashboard.putNumber(TABLE + "kP_Rot", SwerveConstants.PathRotationPID.kP);
//     SmartDashboard.putNumber(TABLE + "kD_Rot", SwerveConstants.PathRotationPID.kD);
//     SmartDashboard.putNumber(TABLE + "Distance_m", 2.5);
//     SmartDashboard.putNumber(TABLE + "RotationDeg", 90.0);
//     SmartDashboard.putNumber(TABLE + "MaxVel_mps", 3.0);
//     SmartDashboard.putNumber(TABLE + "MaxAccel_mps2", 3.0);
//   }

//   /** Una prueba (ida y vuelta) con las ganancias que esten en el dashboard. */
//   public Command manualTrialCommand() {
//     return Commands.sequence(
//             captureAnchor(),
//             trial(
//                 this::dashboardGains,
//                 () -> SmartDashboard.getNumber(TABLE + "RotationDeg", 90.0)),
//             Commands.runOnce(() -> publishResult("Manual", stats.cost(), dashboardGains())))
//         .withName("PathTuning Manual Trial");
//   }

//   /**
//    * Autotune completo por busqueda coordenada: kP traslacion -> kP rotacion -> kD traslacion -> kD
//    * rotacion. Cada candidato corre un path de ida y uno de vuelta.
//    */
//   public Command autoTuneCommand() {
//     return Commands.sequence(
//             captureAnchor(),
//             Commands.runOnce(
//                 () -> {
//                   double[] start = dashboardGains();
//                   System.arraycopy(start, 0, bestGains, 0, 4);
//                 }),
//             stage("kP Traslacion", KP_TRANS, TRANSLATION_KP_CANDIDATES, 0.0),
//             stage("kP Rotacion", KP_ROT, ROTATION_KP_CANDIDATES, ROTATION_TEST_DEG),
//             stage("kD Traslacion", KD_TRANS, KD_CANDIDATES, 0.0),
//             stage("kD Rotacion", KD_ROT, KD_CANDIDATES, ROTATION_TEST_DEG),
//             Commands.runOnce(this::finishAutoTune))
//         .withName("PathTuning AutoTune");
//   }

//   /** Resultado de una prueba (ida + vuelta) con unas ganancias dadas. */
//   public record Resultado(
//       double[] ganancias,
//       double rotacionDeg,
//       double costo,
//       double rmsPosM,
//       double maxPosM,
//       double finalPosM,
//       double rmsHeadingRad,
//       boolean abortado) {}

//   /**
//    * Evalua cada juego de ganancias [kPTrans, kDTrans, kPRot, kDRot] con el mismo path y entrega
//    * el resultado de cada uno, en orden. Es la pieza del autotune robusto: el que decide que
//    * evaluar y como combinar los escenarios es quien llama (ver tools/robust_autotune.py).
//    */
//   public Command evaluarCommand(
//       List<double[]> ganancias, double rotacionDeg, Consumer<Resultado> resultado) {
//     List<Command> pasos = new ArrayList<>();
//     pasos.add(captureAnchor());
//     for (double[] g : ganancias) {
//       double[] copia = g.clone();
//       pasos.add(trial(() -> copia, () -> rotacionDeg));
//       pasos.add(
//           Commands.runOnce(
//               () -> {
//                 publishResult("Evaluar", stats.cost(), copia);
//                 resultado.accept(
//                     new Resultado(
//                         copia,
//                         rotacionDeg,
//                         stats.cost(),
//                         stats.rmsPos(),
//                         stats.maxPosError,
//                         stats.finalPos(),
//                         stats.rmsHeading(),
//                         stats.aborted));
//               }));
//     }
//     return Commands.sequence(pasos.toArray(Command[]::new)).withName("PathTuning Evaluar");
//   }

//   // ---------------------------------------------------------------------------------------------

//   private Command stage(String name, int index, double[] candidates, double rotationDeg) {
//     List<Command> steps = new ArrayList<>();
//     steps.add(
//         Commands.runOnce(
//             () -> {
//               bestStageCost = Double.POSITIVE_INFINITY;
//               SmartDashboard.putString(TABLE + "Stage", name);
//             }));

//     for (double candidate : candidates) {
//       steps.add(Commands.runOnce(() -> candidateValue = candidate));
//       steps.add(trial(() -> candidateGains(index), () -> rotationDeg));
//       steps.add(
//           Commands.runOnce(
//               () -> {
//                 double cost = stats.cost();
//                 publishResult(name + " = " + candidateValue, cost, candidateGains(index));
//                 if (cost < bestStageCost) {
//                   bestStageCost = cost;
//                   bestGains[index] = candidateValue;
//                 }
//               }));
//     }
//     return Commands.sequence(steps.toArray(Command[]::new));
//   }

//   private double[] candidateGains(int index) {
//     double[] gains = bestGains.clone();
//     gains[index] = candidateValue;
//     return gains;
//   }

//   /** Ida al punto lejano y vuelta al ancla, acumulando error en {@link #stats}. */
//   private Command trial(Supplier<double[]> gains, Supplier<Double> rotationDeg) {
//     return Commands.sequence(
//         Commands.runOnce(stats::reset),
//         leg(true, gains, rotationDeg),
//         stop(),
//         Commands.waitSeconds(SETTLE_SECONDS),
//         leg(false, gains, rotationDeg),
//         stop(),
//         Commands.waitSeconds(SETTLE_SECONDS));
//   }

//   private Command leg(boolean towardFar, Supplier<double[]> gains, Supplier<Double> rotationDeg) {
//     return Commands.defer(
//         () -> {
//           RobotConfig config = drivetrain.getRobotConfig();
//           if (config == null) {
//             stats.aborted = true;
//             return Commands.print(
//                 "PathTuning: no hay RobotConfig (falta deploy/pathplanner/settings.json)");
//           }

//           Pose2d current = drivetrain.getState().Pose;
//           Pose2d target = towardFar ? farPose(rotationDeg.get()) : anchor;
//           Rotation2d travel = target.getTranslation().minus(current.getTranslation()).getAngle();

//           PathPlannerPath path =
//               new PathPlannerPath(
//                   PathPlannerPath.waypointsFromPoses(
//                       new Pose2d(current.getTranslation(), travel),
//                       new Pose2d(target.getTranslation(), travel)),
//                   constraints(),
//                   new IdealStartingState(0.0, current.getRotation()),
//                   new GoalEndState(0.0, target.getRotation()));
//           path.preventFlipping = true;

//           // Si la ida se aborto por inestable, regresa con las ganancias seguras de SwerveConstants
//           double[] g = (!towardFar && stats.aborted) ? safeGains() : gains.get();
//           Command follow =
//               new FollowPathCommand(
//                   path,
//                   () -> drivetrain.getState().Pose,
//                   drivetrain::getChassisSpeeds,
//                   drivetrain::applyPathPlannerOutput,
//                   new PPHolonomicDriveController(
//                       new PIDConstants(g[KP_TRANS], 0.0, g[KD_TRANS]),
//                       new PIDConstants(g[KP_ROT], 0.0, g[KD_ROT])),
//                   config,
//                   () -> false);

//           return follow
//               .until(() -> stats.legMaxPosError > ABORT_ERROR_METERS)
//               .beforeStarting(
//                   () -> {
//                     stats.legMaxPosError = 0;
//                     drivetrain.setPathTargetListener(
//                         pose -> stats.sample(pose, drivetrain.getState().Pose));
//                   })
//               .finallyDo(
//                   () -> {
//                     drivetrain.setPathTargetListener(null);
//                     if (stats.legMaxPosError > ABORT_ERROR_METERS) {
//                       stats.aborted = true;
//                     }
//                     stats.addFinal(target, drivetrain.getState().Pose);
//                   });
//         },
//         Set.of(drivetrain));
//   }

//   private Command stop() {
//     return drivetrain.runOnce(
//         () -> drivetrain.applyPathPlannerOutput(new ChassisSpeeds(), DriveFeedforwards.zeros(4)));
//   }

//   private Command captureAnchor() {
//     return Commands.runOnce(() -> anchor = drivetrain.getState().Pose);
//   }

//   /** Punto a {@code Distance_m} al frente del robot (segun su heading al capturar el ancla). */
//   private Pose2d farPose(double rotationDeg) {
//     double distance = SmartDashboard.getNumber(TABLE + "Distance_m", 2.5);
//     Translation2d offset = new Translation2d(distance, anchor.getRotation());
//     return new Pose2d(
//         anchor.getTranslation().plus(offset),
//         anchor.getRotation().plus(Rotation2d.fromDegrees(rotationDeg)));
//   }

//   private PathConstraints constraints() {
//     return new PathConstraints(
//         SmartDashboard.getNumber(TABLE + "MaxVel_mps", 3.0),
//         SmartDashboard.getNumber(TABLE + "MaxAccel_mps2", 3.0),
//         Units.degreesToRadians(360),
//         Units.degreesToRadians(540));
//   }

//   private static double[] safeGains() {
//     return new double[] {
//       SwerveConstants.PathTranslationPID.kP,
//       SwerveConstants.PathTranslationPID.kD,
//       SwerveConstants.PathRotationPID.kP,
//       SwerveConstants.PathRotationPID.kD
//     };
//   }

//   private double[] dashboardGains() {
//     return new double[] {
//       SmartDashboard.getNumber(TABLE + "kP_Trans", SwerveConstants.PathTranslationPID.kP),
//       SmartDashboard.getNumber(TABLE + "kD_Trans", SwerveConstants.PathTranslationPID.kD),
//       SmartDashboard.getNumber(TABLE + "kP_Rot", SwerveConstants.PathRotationPID.kP),
//       SmartDashboard.getNumber(TABLE + "kD_Rot", SwerveConstants.PathRotationPID.kD)
//     };
//   }

//   private void publishResult(String label, double cost, double[] gains) {
//     SmartDashboard.putString(TABLE + "Last/Label", label);
//     SmartDashboard.putNumber(TABLE + "Last/Cost", cost);
//     SmartDashboard.putNumber(TABLE + "Last/RMS_Pos_cm", stats.rmsPos() * 100.0);
//     SmartDashboard.putNumber(TABLE + "Last/Max_Pos_cm", stats.maxPosError * 100.0);
//     SmartDashboard.putNumber(TABLE + "Last/Final_Pos_cm", stats.finalPos() * 100.0);
//     SmartDashboard.putNumber(TABLE + "Last/RMS_Heading_deg", Math.toDegrees(stats.rmsHeading()));
//     SmartDashboard.putNumber(TABLE + "Last/Final_Heading_deg", Math.toDegrees(stats.finalHeading()));
//     SmartDashboard.putBoolean(TABLE + "Last/Aborted", stats.aborted);
//     System.out.printf(
//         "[PathTuning] %-24s kPt=%.2f kDt=%.2f kPr=%.2f kDr=%.2f | costo=%.4f rms=%.1fcm max=%.1fcm"
//             + " final=%.1fcm head=%.1fdeg%s%n",
//         label,
//         gains[KP_TRANS],
//         gains[KD_TRANS],
//         gains[KP_ROT],
//         gains[KD_ROT],
//         cost,
//         stats.rmsPos() * 100.0,
//         stats.maxPosError * 100.0,
//         stats.finalPos() * 100.0,
//         Math.toDegrees(stats.rmsHeading()),
//         stats.aborted ? " ABORTADO" : "");
//   }

//   private void finishAutoTune() {
//     SmartDashboard.putString(TABLE + "Stage", "Terminado");
//     SmartDashboard.putNumber(TABLE + "kP_Trans", bestGains[KP_TRANS]);
//     SmartDashboard.putNumber(TABLE + "kD_Trans", bestGains[KD_TRANS]);
//     SmartDashboard.putNumber(TABLE + "kP_Rot", bestGains[KP_ROT]);
//     SmartDashboard.putNumber(TABLE + "kD_Rot", bestGains[KD_ROT]);
//     System.out.printf(
//         "[PathTuning] RESULTADO -> pega en SwerveConstants:%n"
//             + "  PathTranslationPID = new PIDConstants(%.2f, 0.0, %.2f);%n"
//             + "  PathRotationPID = new PIDConstants(%.2f, 0.0, %.2f);%n",
//         bestGains[KP_TRANS],
//         bestGains[KD_TRANS],
//         bestGains[KP_ROT],
//         bestGains[KD_ROT]);
//   }

//   /** Acumulador del error de seguimiento de una prueba (ida + vuelta). */
//   private static final class TrackingStats {
//     double sumSqPos;
//     double sumSqHeading;
//     int samples;
//     double maxPosError;
//     double legMaxPosError;
//     double sumFinalPos;
//     double sumFinalHeading;
//     int legs;
//     boolean aborted;

//     void reset() {
//       sumSqPos = 0;
//       sumSqHeading = 0;
//       samples = 0;
//       maxPosError = 0;
//       legMaxPosError = 0;
//       sumFinalPos = 0;
//       sumFinalHeading = 0;
//       legs = 0;
//       aborted = false;
//     }

//     void sample(Pose2d target, Pose2d actual) {
//       double posError = target.getTranslation().getDistance(actual.getTranslation());
//       double headingError = headingError(target, actual);
//       sumSqPos += posError * posError;
//       sumSqHeading += headingError * headingError;
//       samples++;
//       maxPosError = Math.max(maxPosError, posError);
//       legMaxPosError = Math.max(legMaxPosError, posError);
//     }

//     void addFinal(Pose2d target, Pose2d actual) {
//       sumFinalPos += target.getTranslation().getDistance(actual.getTranslation());
//       sumFinalHeading += headingError(target, actual);
//       legs++;
//     }

//     double rmsPos() {
//       return samples == 0 ? 0 : Math.sqrt(sumSqPos / samples);
//     }

//     double rmsHeading() {
//       return samples == 0 ? 0 : Math.sqrt(sumSqHeading / samples);
//     }

//     double finalPos() {
//       return legs == 0 ? 0 : sumFinalPos / legs;
//     }

//     double finalHeading() {
//       return legs == 0 ? 0 : sumFinalHeading / legs;
//     }

//     /** Metros equivalentes: 1 rad de heading pesa como 15 cm de posicion. */
//     double cost() {
//       if (aborted || samples == 0) {
//         return ABORTED_COST;
//       }
//       return rmsPos()
//           + 0.25 * maxPosError
//           + finalPos()
//           + 0.15 * (rmsHeading() + finalHeading());
//     }

//     private static double headingError(Pose2d target, Pose2d actual) {
//       return Math.abs(
//           MathUtil.angleModulus(
//               target.getRotation().getRadians() - actual.getRotation().getRadians()));
//     }
//   }
// }
