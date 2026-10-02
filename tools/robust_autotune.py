#!/usr/bin/env python3
"""Autotune ROBUSTO del PID de trayectoria (PPHolonomicDriveController).

El autotune normal busca las ganancias que mejor siguen el path en UN robot: el
nominal. Este evalua una cuadricula fija de ganancias contra varios robots
posibles --- masa, agarre de la alfombra, voltaje de bateria --- y se queda con
la que tiene el MEJOR PEOR CASO (minimax): la que nunca sale mal, aunque en el
robot nominal haya otra un poco mejor.

Necesita el lockstep de MARS (cada corrida sale igual, ±0.6%): sin el, el ruido
de ±10% del tiempo real taparia las diferencias entre escenarios.

    python tools/robust_autotune.py            # 5 mundos x 2 baterias, ~25 min
    python tools/robust_autotune.py --rapido   # 2 mundos x 1 bateria, para probar

Dos pasadas:
  1. traslacion: kP x kD en un path recto, rotacion fija en SwerveConstants.
  2. rotacion:   kP x kD en un path que gira 90 grados, con la traslacion
                 robusta de la pasada 1.

Lo que cambia cada escenario y donde:
  masa y mu  -> el mundo de Gazebo (gen_offseason2026.py --masa --mu --sufijo).
                El robot NO se entera: PathPlanner sigue creyendo 58 kg y mu 0.9,
                que es justo lo que pasa en la cancha.
  bateria    -> RoboRioSim.setVInVoltage(): el TalonFX satura en ese voltaje.

Rutas (variables de entorno, con los valores de esta maquina por defecto):
  MARS_SIM_DIR  simulationstudio de MARS
  MARS_CONDA    entorno conda mars-sim
  WPILIB_JDK    JDK de WPILib (el de Adoptium hace tronar el HAL)
"""
import argparse
import datetime
import itertools
import json
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SIM = Path(os.environ.get(
    "MARS_SIM_DIR", r"C:\PROGRAMACION\STZROBOTICS\MARSAPP\mars-environment\simulationstudio"))
CONDA = Path(os.environ.get("MARS_CONDA", Path.home() / "miniforge3" / "envs" / "mars-sim"))
JDK = os.environ.get("WPILIB_JDK", r"C:\Users\Public\wpilib\2026\jdk")
PUERTO = 5811
ROBOT_MAP = "models/offseason2026/robot-map.json"

# --- los escenarios ------------------------------------------------------------
# Las cuatro esquinas de masa x agarre, mas el nominal. Las esquinas son donde
# vive el peor caso: el robot pesado en alfombra resbalosa frena tarde, el
# ligero en alfombra con agarre reacciona brusco.
MUNDOS_COMPLETO = [(58, 0.9), (54, 0.8), (54, 1.0), (62, 0.8), (62, 1.0)]
# 12.5 V es una bateria cargada; 10.5 V es una bateria cansada bajo carga.
VOLTAJES_COMPLETO = [12.5, 10.5]

MUNDOS_RAPIDO = [(58, 0.9), (62, 0.8)]
VOLTAJES_RAPIDO = [12.0]

# --- las cuadriculas ------------------------------------------------------------
KP_TRASLACION = [4, 6, 8, 10, 12, 14, 16, 20, 24]
KD_TRASLACION = [0.0, 0.2, 0.4]
KP_ROTACION = [2, 4, 6, 8, 10]
KD_ROTACION = [0.0, 0.2]
# La rotacion durante la pasada de traslacion: la de SwerveConstants.
ROTACION_FIJA = (5.0, 0.0)


def entorno_hijos():
    """PATH con conda primero y sin MSYS2, como Supervisor::path_para_hijos."""
    env = dict(os.environ)
    resto = [p for p in env.get("PATH", "").split(";") if p and "msys64" not in p.lower()]
    env["PATH"] = ";".join([str(CONDA / "Library" / "bin")] + resto)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(SIM / "build")
    env["GZ_SIM_RESOURCE_PATH"] = str(SIM / "models")
    return env


def matar_simulacion():
    subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         "Get-Process mars-bridge,mars-sim-server,mars-sim-gui -ErrorAction SilentlyContinue"
         " | Stop-Process -Force"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def bridge_exe():
    for p in (SIM / "build" / "mars-bridge.exe",
              SIM / "bridge" / "target" / "release" / "mars-bridge.exe"):
        if p.is_file():
            return p
    sys.exit("falta mars-bridge.exe (cargo build --release en simulationstudio/bridge)")


def generar_mundo(masa, mu):
    sufijo = f"m{masa:g}_mu{mu:g}"
    subprocess.run(
        [str(CONDA / "python.exe"), "gen_offseason2026.py",
         "--masa", str(masa), "--mu", str(mu), "--sufijo", sufijo],
        cwd=SIM / "worlds", check=True, stdout=subprocess.DEVNULL)
    return sufijo, f"worlds/offseason2026_{sufijo}_lockstep.sdf"


def levantar_simulacion(mundo, logs, gui):
    """Servidor con `run` (MarsLink espera cada comando) + bridge en lockstep.

    stdout a archivo y NO a un pipe: los hijos de Gazebo heredan los handles, y
    un pipe abierto por ellos dejaria a este script esperando para siempre.
    """
    env = entorno_hijos()
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    motor = subprocess.Popen(
        [str(SIM / "build" / "mars-sim-server.exe"), mundo, "run"], cwd=SIM, env=env,
        stdout=open(logs / "motor.log", "a"), stderr=subprocess.STDOUT, creationflags=flags)
    time.sleep(1.5)
    if gui:
        env_gui = dict(env)
        env_gui["OGRE2_RESOURCE_PATH"] = str(CONDA / "Library" / "bin" / "OGRE-Next")
        env_gui["GZ_GUI_PLUGIN_PATH"] = ";".join([
            str(CONDA / "Library" / "lib" / "gz-gui-10" / "plugins"),
            str(CONDA / "Library" / "lib" / "gz-sim-10" / "plugins" / "gui")])
        subprocess.Popen([str(SIM / "build" / "mars-sim-gui.exe"), "gui/mars.config"], cwd=SIM,
                         env=env_gui, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    puente = subprocess.Popen(
        [str(bridge_exe()), "--robot-map", ROBOT_MAP, "--lockstep-port", str(PUERTO)],
        cwd=SIM, env=env, stdout=open(logs / "bridge.log", "a"), stderr=subprocess.STDOUT,
        creationflags=flags)
    time.sleep(1.5)
    if motor.poll() is not None or puente.poll() is not None:
        sys.exit(f"la simulacion no arranco; ver {logs}")


def correr_escenario(escenario, ganancias, rotacion, voltajes, salida, logs):
    env = dict(os.environ)
    env.update({
        "JAVA_HOME": JDK,
        "MARS_LOCKSTEP": f"127.0.0.1:{PUERTO}",
        "ROBUST_SALIDA": str(salida),
        "ROBUST_ESCENARIO": escenario,
        "ROBUST_ROTACION": str(rotacion),
        "ROBUST_VOLTAJES": ",".join(str(v) for v in voltajes),
        "ROBUST_GANANCIAS": ";".join(",".join(str(x) for x in g) for g in ganancias),
    })
    gradle = REPO / ("gradlew.bat" if os.name == "nt" else "gradlew")
    with open(logs / f"gradle_{escenario}.log", "w") as log:
        r = subprocess.run(
            [str(gradle), "cleanTest", "test", "--offline", "--tests", "*RobustTuneRunner*", "--info"],
            cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        sys.exit(f"el runner fallo en {escenario}; ver {logs / f'gradle_{escenario}.log'}")


def pasada(nombre, ganancias, rotacion, mundos, voltajes, carpeta, gui):
    salida = carpeta / f"{nombre}.jsonl"
    print(f"\n== pasada {nombre}: {len(ganancias)} candidatos x {len(mundos)} mundos x "
          f"{len(voltajes)} baterias", flush=True)
    for masa, mu in mundos:
        t0 = time.time()
        escenario, mundo = generar_mundo(masa, mu)
        matar_simulacion()
        levantar_simulacion(mundo, carpeta, gui)
        try:
            correr_escenario(escenario, ganancias, rotacion, voltajes, salida, carpeta)
        finally:
            matar_simulacion()
            (SIM / mundo).unlink(missing_ok=True)
        print(f"   {escenario}: {time.time() - t0:.0f} s", flush=True)
    return [json.loads(l) for l in salida.read_text().splitlines() if l.strip()]


def elegir(resultados, claves):
    """Minimax: la ganancia con el menor peor costo; empata el promedio."""
    por = {}
    for r in resultados:
        clave = tuple(r[k] for k in claves)
        por.setdefault(clave, []).append(r)
    filas = []
    for clave, rs in por.items():
        costos = [r["costo"] for r in rs]
        peor = max(rs, key=lambda r: r["costo"])
        nominal = [r["costo"] for r in rs if r["escenario"] == "m58_mu0.9"]
        filas.append({
            "ganancias": clave,
            "peor": max(costos),
            "promedio": statistics.mean(costos),
            "nominal": min(nominal) if nominal else float("nan"),
            "peor_escenario": f"{peor['escenario']} @ {peor['voltaje']:g} V",
            "abortos": sum(r["abortado"] for r in rs),
        })
    filas.sort(key=lambda f: (f["peor"], f["promedio"]))
    return filas


def tabla(filas, claves, n=10):
    lineas = [f"| {' | '.join(claves)} | peor | promedio | nominal | peor escenario | abortos |",
              "|" + "---|" * (len(claves) + 5)]
    for f in filas[:n]:
        g = " | ".join(f"{x:g}" for x in f["ganancias"])
        lineas.append(f"| {g} | {f['peor']:.4f} | {f['promedio']:.4f} | {f['nominal']:.4f} | "
                      f"{f['peor_escenario']} | {f['abortos']} |")
    return "\n".join(lineas)


def main():
    cli = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    cli.add_argument("--rapido", action="store_true", help="2 mundos x 1 bateria, para probar")
    cli.add_argument("--gui", action="store_true", help="abrir la ventana de Gazebo")
    args = cli.parse_args()

    mundos = MUNDOS_RAPIDO if args.rapido else MUNDOS_COMPLETO
    voltajes = VOLTAJES_RAPIDO if args.rapido else VOLTAJES_COMPLETO
    carpeta = REPO / "tools" / "robust_out" / datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta.mkdir(parents=True)
    print(f"resultados en {carpeta}")

    # --- pasada 1: traslacion ---
    candidatos = [(kp, kd, *ROTACION_FIJA) for kp, kd in itertools.product(KP_TRASLACION, KD_TRASLACION)]
    res = pasada("traslacion", candidatos, 0.0, mundos, voltajes, carpeta, args.gui)
    filas_t = elegir(res, ("kPt", "kDt"))
    kpt, kdt = filas_t[0]["ganancias"]
    mejor_nominal_t = min(filas_t, key=lambda f: f["nominal"])
    print("\n" + tabla(filas_t, ["kP", "kD"]))

    # --- pasada 2: rotacion, con la traslacion robusta ---
    candidatos = [(kpt, kdt, kp, kd) for kp, kd in itertools.product(KP_ROTACION, KD_ROTACION)]
    res = pasada("rotacion", candidatos, 90.0, mundos, voltajes, carpeta, args.gui)
    filas_r = elegir(res, ("kPr", "kDr"))
    kpr, kdr = filas_r[0]["ganancias"]
    mejor_nominal_r = min(filas_r, key=lambda f: f["nominal"])
    print("\n" + tabla(filas_r, ["kP", "kD"]))

    resumen = f"""# Autotune robusto del PID de trayectoria

{len(mundos)} mundos (masa kg, mu): {', '.join(f'({m}, {u})' for m, u in mundos)}
Baterias: {', '.join(f'{v} V' for v in voltajes)}
Criterio: el menor PEOR costo entre todos los escenarios (minimax).

## Traslacion (path recto)

{tabla(filas_t, ["kP", "kD"])}

Mejor en el robot nominal: kP {mejor_nominal_t['ganancias'][0]:g}, kD {mejor_nominal_t['ganancias'][1]:g}
(nominal {mejor_nominal_t['nominal']:.4f}, pero peor caso {mejor_nominal_t['peor']:.4f}).

## Rotacion (path con 90 grados)

{tabla(filas_r, ["kP", "kD"])}

Mejor en el robot nominal: kP {mejor_nominal_r['ganancias'][0]:g}, kD {mejor_nominal_r['ganancias'][1]:g}
(nominal {mejor_nominal_r['nominal']:.4f}, pero peor caso {mejor_nominal_r['peor']:.4f}).

## Resultado

```java
public static final PIDConstants PathTranslationPID = new PIDConstants({kpt:g}, 0.0, {kdt:g});
public static final PIDConstants PathRotationPID = new PIDConstants({kpr:g}, 0.0, {kdr:g});
```
"""
    (carpeta / "resumen.md").write_text(resumen, encoding="utf-8")
    print(resumen)


if __name__ == "__main__":
    main()
