// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Set;

/**
 * Botones de SmartDashboard/Elastic para caracterizar y tunear el chasis sin ocupar botones del
 * control. Todos requieren el robot habilitado (teleop) y mueven el robot solo.
 *
 * <ul>
 *   <li>SysId/*: rutinas de CTRE. Graban a un .hoot con SignalLogger; se extraen con Tuner X ->
 *       Log Extractor -> wpilog y se analizan en la app SysId de WPILib.
 *   <li>PathTuning/*: prueba manual y autotune del PID de PathPlanner ({@link PathFollowingTuner}).
 * </ul>
 */
public final class ChassisTuningDashboard {

  private ChassisTuningDashboard() {}

  public static void publish(CommandSwerveDrivetrain drivetrain) {
    SendableChooser<SysIdRoutine> routine = new SendableChooser<>();
    routine.setDefaultOption("Translation (drive kS/kV/kA)", drivetrain.sysIdManager.m_sysIdRoutineTranslation);
    routine.addOption("Steer (azimuth)", drivetrain.sysIdManager.m_sysIdRoutineSteer);
    routine.addOption("Rotation (heading)", drivetrain.sysIdManager.m_sysIdRoutineRotation);
    SmartDashboard.putData("SysId/Routine", routine);

    SmartDashboard.putData(
        "SysId/1 Quasistatic Forward",
        sysId(drivetrain, routine, true, SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "SysId/2 Quasistatic Reverse",
        sysId(drivetrain, routine, true, SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        "SysId/3 Dynamic Forward",
        sysId(drivetrain, routine, false, SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "SysId/4 Dynamic Reverse",
        sysId(drivetrain, routine, false, SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        "SysId/5 Stop Logger",
        Commands.runOnce(SignalLogger::stop).ignoringDisable(true).withName("Stop SignalLogger"));

    PathFollowingTuner tuner = new PathFollowingTuner(drivetrain);
    SmartDashboard.putData("PathTuning/Run Manual Trial", tuner.manualTrialCommand());
    SmartDashboard.putData("PathTuning/Run AutoTune", tuner.autoTuneCommand());
  }

  private static Command sysId(
      CommandSwerveDrivetrain drivetrain,
      SendableChooser<SysIdRoutine> routine,
      boolean quasistatic,
      SysIdRoutine.Direction direction) {
    return Commands.defer(
            () -> {
              drivetrain.setSysIdRoutine(routine.getSelected());
              return quasistatic
                  ? drivetrain.sysIdQuasistatic(direction)
                  : drivetrain.sysIdDynamic(direction);
            },
            Set.of(drivetrain))
        .beforeStarting(SignalLogger::start)
        .withName((quasistatic ? "Quasistatic " : "Dynamic ") + direction);
  }
}
