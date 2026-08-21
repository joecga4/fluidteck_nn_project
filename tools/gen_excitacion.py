#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
gen_excitacion.py — Disenia la secuencia de excitacion para identificar la planta.

============================ GUIA DE LECTURA ============================
Por que este archivo existe
---------------------------
La leccion mas cara del proyecto hermano (`pi5_qnx_project`) fue esta: **la
calidad del modelo la fija la EXCITACION, no la red**. Alla hubo que recapturar
todo porque los escalones eran demasiado cortos y solo el 29% de las muestras
estaba en regimen. Aqui el riesgo es otro (ver mas abajo) pero la moraleja es la
misma: la secuencia se disenia y se valida ANTES de lanzarla contra la maquina.

Los datos de 2017 son el contraejemplo perfecto: cada fichero tiene UN escalon
arriba y UNO abajo (CLAUDE.md §5.1). Con eso no se identifica nada.

Las tres restricciones que hacen este problema distinto
-------------------------------------------------------
1. LA PLANTA ES UN INTEGRADOR (CLAUDE.md §3.3). El comando fija la VELOCIDAD, no
   la posicion. Un tramo de amplitud u y duracion T consume un RECORRIDO
   dx = v(u)*T. Con solo 150 mm de carrera util y 2.6 mm/s de tope al extender
   (el caudal de la bomba satura ahi), la carrera se agota en menos de un minuto
   de comando constante. La secuencia tiene que
   PLEGARSE sobre si misma: cuando se acerca a un extremo, cambia de signo.
   Ese plegado es la diferencia principal frente a una APRBS de libro.

2. EL ENSAYO REAL OCURRE PEGADO AL NULL. Las normas piden 0.1 y 1.5 mm/min, que
   son comandos de -0.365 y -0.304 V: separados 62 mV y a un lado del cruce por
   cero. Si repartimos las amplitudes de forma uniforme casi ninguna muestra cae
   ahi, y el modelo sale excelente donde no importa. Por eso se sortea la
   VELOCIDAD de forma logaritmica y se despeja el comando (`u_para_vel`), no al
   reves: muestrear el comando no garantiza cubrir las velocidades utiles.

3. LA GANANCIA YA ESTA MEDIDA, pero se disenia igual con margen. El plegado
   necesita predecir donde estara el vastago, y esa prediccion usa la ley medida
   (planta.py). Aun asi `daq.py` supervisa la posicion en tiempo real durante la
   emision: una secuencia de 10 minutos nunca se lanza "a ciegas".

Que produce
-----------
    results/excitacion_<etiqueta>.csv   ->  columnas  t[s], u[V]
Ese CSV es lo que se precarga en el buffer del AO (NI 9263) para emitirlo con
temporizacion por hardware (CLAUDE.md §2.5.1).

Uso
---
    # disenia, valida contra el simulador y grafica
    python tools/gen_excitacion.py --seed 1 --etiqueta train --plot
    python tools/gen_excitacion.py --seed 7 --etiqueta val   --plot

    # forzando otro null (p.ej. si se re-mide con el aceite mas caliente)
    python tools/gen_excitacion.py --seed 1 --etiqueta train --u-null -0.41

    # ademas, fisica de dos camaras sobre un trozo (presiones, transitorios)
    python tools/gen_excitacion.py --seed 1 --etiqueta train --sim-completo

Las semillas 1 y 7 dan dos secuencias INDEPENDIENTES: una para entrenar y otra
para validar. No se parte una sola serie en dos (eso sobreestima el ajuste: los
dos trozos comparten tramos y correlaciones).
"""

from __future__ import annotations

import argparse
import csv
import os

import numpy as np


# ============================================================================
# LOS NUMEROS DE LA PLANTA vienen de tools/planta.py, que es la fuente unica.
# ============================================================================
# Estaban duplicados aqui y en daq.py, con nombres distintos. Si se vuelve a
# medir la planta, se cambia en planta.py y todo el proyecto lo sigue.
#
# Lo que importa para disenar la excitacion: el termino independiente de la
# ley medida NO es cero. Hay deriva a comando nulo, asi que la velocidad cero
# se consigue en U_NULL ~ -0.37 V. Una secuencia disenada suponiendo el null
# en el origen deriva hacia el positivo y se sale de la ventana: se comprobo,
# y el 47 % de las muestras acababa fuera.
from planta import (K_POS, B_POS, K_NEG, B_NEG, U_NULL,
                    vel_medida, u_para_vel)

# ============================================================================
# 1. DISENIO DE LA SECUENCIA
# ============================================================================
def disenia_secuencia(
    dur_total: float = 600.0,
    Ts: float = 0.001,
    u_max: float = 10.0,
    v_min: float = 0.05 / 60.0,   # [mm/s] = 0.05 mm/min, media decada por
    v_max: float = 4.0,           #         debajo del ensayo de viga
    x0: float = 75.0,
    x_lo: float = 25.0,
    x_hi: float = 125.0,
    dx_obj: float = 8.0,
    t_rap_min: float = 0.05,
    t_rap_max: float = 0.60,
    t_reg_min: float = 1.0,
    t_reg_max: float = 25.0,
    p_cero: float = 0.10,
    p_rapido: float = 0.55,
    u_null: float = U_NULL,
    seed: int = 1,
) -> dict:
    """Construye la secuencia APRBS plegada dentro de la ventana de posicion.

    TRES CLASES DE TRAMO — el nucleo del disenio
    ---------------------------------------------
    Un unico criterio de duracion no puede servir a la vez a los transitorios y
    al regimen lento, y mezclarlos mal fue el error de la primera captura del
    proyecto del motor. Aqui se separan de forma explicita:

      RAPIDO  (p_rapido)  duracion U(t_rap_min, t_rap_max), corta.
                          Ensenia la DINAMICA: como responde el caudal al
                          cambiar el comando de golpe. Son los tramos donde una
                          red le gana a un PID (anticipacion).
      REGIMEN (resto)     duracion por presupuesto de recorrido,
                          T = dx_obj/|v(u)| acotada a [t_reg_min, t_reg_max].
                          Ensenia el MAPA ESTATICO comando->velocidad. Como la
                          duracion sale del recorrido, las amplitudes pequenias
                          reciben tramos LARGOS (que es justo lo que hace falta
                          para medir 0.27 mm/min por encima del ruido del
                          sensor) y las grandes tramos cortos (no agotan la
                          carrera).
      CERO    (p_cero)    comando exactamente 0. Miden la DERIVA DE NULL (§5.1)
                          repartida por toda la captura, no solo al principio:
                          si el null deriva con la temperatura del aceite, esto
                          lo destapa.

    Parametros
    ----------
    dur_total : duracion objetivo [s]
    Ts        : periodo de muestreo del AO [s] (1 ms = 1 kHz)
    u_max     : fondo de escala del comando [V]
    v_min,v_max : rango de VELOCIDADES a cubrir [mm/s]. v_min baja media decada
                por debajo del ensayo de viga para que esa banda quede poblada.
    u_null    : comando de velocidad nula sobre el que centrar el disenio.
                Por defecto el medido; se cambia si el null se ha movido.
    x0        : posicion inicial del vastago [mm]
    x_lo,x_hi : ventana de posicion SEGURA [mm]. Estrictamente interior a los
                topes: la carrera util del actuador es 0..150 mm.
    dx_obj    : recorrido objetivo por tramo de regimen [mm].
    seed      : semilla. Misma semilla -> misma secuencia (reproducible).

    Devuelve un dict con la secuencia muestreada y los tramos que la componen.
    """
    rng = np.random.default_rng(seed)

    tramos: list[tuple[float, float, str]] = []   # (amplitud [V], duracion [s], clase)
    x = float(x0)                                 # posicion PREDICHA [mm]
    x_c = 0.5 * (x_lo + x_hi)                     # centro de la ventana
    t_acum = 0.0
    t_pos = 0.0                                   # tiempo acumulado con u>0
    t_neg = 0.0                                   # idem con u<0

    while t_acum < dur_total:
        # ---- 1. clase y amplitud -----------------------------------------
        r = rng.random()
        if r < p_cero:
            clase, v_obj = "cero", None
        else:
            clase = "rapido" if r < p_cero + p_rapido else "regimen"
            # Reparto LOGARITMICO EN VELOCIDAD, no en comando: asi cada decada
            # de velocidad recibe el mismo numero de muestras y las bandas de
            # los ensayos normados (0.1 y 1.5 mm/min) quedan pobladas por
            # construccion (§6.1.1).
            lo, hi = np.log10(v_min), np.log10(v_max)
            v_obj = float(10 ** rng.uniform(lo, hi))

        # ---- 2. signo: aqui es donde se PLIEGA la secuencia ---------------
        # Dos fuerzas se combinan: volver al centro de la ventana (que es lo que
        # impide chocar contra un tope) y equilibrar el tiempo en cada sentido
        # (que es lo que impide un modelo sesgado hacia un signo).
        margen = 0.25 * (x_hi - x_lo)
        if x > x_hi - margen:
            signo = -1.0
        elif x < x_lo + margen:
            signo = +1.0
        else:
            sesgo_pos = 0.5 - 0.35 * (x - x_c) / (0.5 * (x_hi - x_lo))
            # Correccion de balance: si ya se ha pasado mas tiempo en un signo,
            # el otro se vuelve mas probable.
            if t_pos + t_neg > 0:
                desbal = (t_pos - t_neg) / (t_pos + t_neg)
                sesgo_pos -= 0.30 * desbal
            signo = +1.0 if rng.random() < np.clip(sesgo_pos, 0.05, 0.95) else -1.0
        # ---- 2b. DE VELOCIDAD OBJETIVO A COMANDO -------------------------
        # Los tramos "cero" se dejan a 0 V EXACTOS a proposito: siguen midiendo
        # la deriva a lo largo de toda la captura (que con el peso colgando y el
        # null con fuga NO es cero, sino ~ +0.14 mm/s).
        if clase == "cero":
            u = 0.0
        else:
            # `u_para_vel` despeja el comando con el null MEDIDO. Si se pide
            # disenar para otro null (el aceite mas caliente, otra sesion), se
            # traslada el resultado: se conserva el reparto de velocidades y se
            # mueve el centro. Sin esto la opcion no hacia absolutamente nada.
            u = u_para_vel(signo * v_obj, u_max) + (u_null - U_NULL)
            u = float(np.clip(u, -u_max, u_max))

        # ---- 3. duracion segun la clase ----------------------------------
        v_pred = float(vel_medida(u))
        if clase == "cero":
            T = float(rng.uniform(0.3, 2.0))
        elif clase == "rapido":
            T = float(rng.uniform(t_rap_min, t_rap_max))
        else:
            T = float(np.clip(dx_obj / max(abs(v_pred), 1e-4),
                              t_reg_min, t_reg_max))

        # ---- 4. recorte final: nunca predecir una salida de la ventana ----
        dx = v_pred * T
        if v_pred > 0 and x + dx > x_hi:
            T = max(0.0, (x_hi - x) / v_pred)
        elif v_pred < 0 and x + dx < x_lo:
            T = max(0.0, (x_lo - x) / v_pred)
        if T < Ts:
            continue

        x += v_pred * T
        tramos.append((u, T, clase))
        t_acum += T
        if v_pred > 0:
            t_pos += T
        elif v_pred < 0:
            t_neg += T

    # ---- muestreo a Ts ---------------------------------------------------
    trozos = [np.full(max(1, int(round(T / Ts))), u) for u, T, _ in tramos]
    u_s = np.concatenate(trozos)
    n = len(u_s)
    t_s = np.arange(n) * Ts

    return {
        "t": t_s,
        "u": u_s,
        "tramos": tramos,
        "Ts": Ts,
        "x0": x0,
        "ventana": (x_lo, x_hi),
        "seed": seed,
    }


# ============================================================================
# 2. INFORME DE COBERTURA
# ============================================================================
def informe(sec: dict) -> None:
    """Resume si la secuencia cubre lo que tiene que cubrir. Se lee ANTES de
    llevarla a la maquina."""
    u, Ts = sec["u"], sec["Ts"]
    tramos = sec["tramos"]
    n = len(u)
    au = np.abs(u)

    print("=" * 72)
    print(f"SECUENCIA DE EXCITACION  (semilla {sec['seed']})")
    print("=" * 72)
    print(f"  duracion       : {n*Ts:8.1f} s  ({n} muestras a Ts={Ts*1e3:.1f} ms)")
    print(f"  tramos         : {len(tramos)}")
    print(f"  inversiones    : {int(np.sum(np.diff(np.sign(u[u != 0])) != 0))}")
    print(f"  comando        : {u.min():+.3f} .. {u.max():+.3f} V")
    for cl in ("rapido", "regimen", "cero"):
        k = sum(1 for _, _, c in tramos if c == cl)
        tt = sum(T for _, T, c in tramos if c == cl)
        print(f"  tramos {cl:8s}: {k:4d} ({k/len(tramos)*100:3.0f}%)  "
              f"tiempo {tt:7.1f} s ({tt/(n*Ts)*100:3.0f}%)")
    print(f"  muestras signo -: {np.sum(u < 0)/n*100:.0f}%   "
          f"signo +: {np.sum(u > 0)/n*100:.0f}%")

    # La cobertura se reporta en VELOCIDAD, no en |u|. Con el null desplazado a
    # -0.37 V, el valor absoluto del comando ya no dice nada: un "u = -0.37 V"
    # es reposo y un "u = 0 V" es +8 mm/min. Lo que hay que cubrir bien es el
    # rango de VELOCIDADES, y en particular la banda de los ensayos normados.
    v = np.abs(vel_medida(u)) * 60.0            # mm/min
    print("\n  COBERTURA POR DECADA DE VELOCIDAD  (lo critico: §6.1.1)")
    print(f"  {'banda de |v| [mm/min]':>26s} {'muestras':>10s} {'%':>8s}")
    bordes = [0, 0.05, 0.2, 1.0, 2.0, 5.0, 20.0, 100.0, 1e9]
    for lo, hi in zip(bordes[:-1], bordes[1:]):
        m = int(np.sum((v >= lo) & (v < hi)))
        marca = ""
        if lo <= 0.1 < hi:
            marca = "  <-- ensayo de VIGA (0.1)"
        if lo <= 1.5 < hi:
            marca = "  <-- ensayo de LOSA (1.5)"
        et = f"{lo:8.2f}-{hi:<8.2f}" if hi < 1e8 else f"{lo:8.2f}+       "
        print(f"  {et} {m:10d} {m/n*100:7.1f}%{marca}")
    print(f"\n  velocidad predicha: {vel_medida(u).min():+.3f} .. "
          f"{vel_medida(u).max():+.3f} mm/s")


# ============================================================================
# 3. VALIDACION CONTRA EL SIMULADOR
# ============================================================================
def _params_sim():
    """Carga ParamsPlanta con no linealidades del orden de lo medido en 2017,
    para que la validacion no salga optimista."""
    try:
        from planta_sim import ParamsPlanta, PlantaHidraulica
    except ImportError:
        import sys
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from planta_sim import ParamsPlanta, PlantaHidraulica
    p = ParamsPlanta()
    p.null_off = 0.004      # deriva a comando cero (medida en 2017: §5.1)
    p.solape = 0.002        # solape del carrete (centro cerrado)
    p.histeresis = 0.004
    p.F_stick = 400.0
    p.F_coul = 250.0
    return p, PlantaHidraulica


def valida_con_simulador(sec: dict, completo: bool = False,
                         usar_medido: bool = True) -> dict | None:
    """Comprueba que la secuencia no saca el vastago de la ventana segura.

    DOS NIVELES, y por defecto el barato:

      CINEMATICO (defecto). Integra  v = K_vel(signo)*u + deriva_null  con las
      ganancias de regimen que resuelve `planta_sim` para cada sentido (que NO
      son iguales: retraer sale ~0.77x, ver §5.4). Para saber DONDE acaba el
      vastago eso es todo lo que hace falta: el envolvente de posicion lo fija
      la ganancia cuasi-estatica, no la dinamica de 300 Hz. Cuesta milisegundos.

      COMPLETO (--sim-completo). Integra la fisica de dos camaras sobre un
      TROZO de la secuencia. Sirve para mirar presiones y transitorios, pero
      simular 600 s con dt_int=5e-5 son 12 millones de pasos de RK4: no se hace
      para una comprobacion de recorrido.
    """
    p, PlantaHidraulica = _params_sim()
    x_lo, x_hi = sec["ventana"]
    Ts = sec["Ts"]
    u = sec["u"]

    # --- cinematico, con las ganancias MEDIDAS ----------------------------
    if usar_medido:
        K_ext, b_ext = K_POS, B_POS
        K_ret, b_ret = K_NEG, B_NEG
        fuente = "MEDIDAS en el equipo (2026-08-12)"
    else:
        K_ext, K_ret = p.K_vel(True) * 1e3, p.K_vel(False) * 1e3
        b_ext = b_ret = p.null_off * K_ext * (p.i_nom / p.K_amp)
        fuente = "del modelo fisico de dos camaras"

    v = np.where(u >= 0, K_ext * u + b_ext, K_ret * u + b_ret)
    x = sec["x0"] + np.cumsum(v) * Ts
    x = np.clip(x, 0.0, p.L_carrera * 1e3)   # topes mecanicos

    print(f"\n  VALIDACION CINEMATICA (ganancias {fuente})")
    print(f"    rama u>0           : v = {K_ext:.4f}*u {b_ext:+.4f}")
    print(f"    rama u<0           : v = {K_ret:.4f}*u {b_ret:+.4f}"
          f"   (asimetria {K_ret/K_ext:.3f})")
    print(f"    deriva a comando 0 : {b_ext:+7.4f} mm/s")
    if usar_medido:
        u_cero = -b_ret / K_ret
        print(f"    comando de velocidad CERO: {u_cero:+.3f} V  <-- el null NO esta en 0")
    print(f"    posicion recorrida : {x.min():7.1f} .. {x.max():7.1f} mm")
    print(f"    ventana pedida     : {x_lo:7.1f} .. {x_hi:7.1f} mm")
    print(f"    carrera mecanica   : {0.0:7.1f} .. {p.L_carrera*1e3:7.1f} mm")
    print(f"    velocidad          : {v.min():7.3f} .. {v.max():7.3f} mm/s")

    fuera = (x < x_lo) | (x > x_hi)
    if fuera.any():
        print(f"    !! {fuera.sum()} muestras ({fuera.mean()*100:.1f}%) FUERA de la ventana")
        print("       -> estrechar la ventana (--xlo/--xhi) o acortar los tramos")
        print("          de regimen (--dx). La secuencia se pliega dentro de la")
        print("          ventana, pero si los tramos son largos puede rebasarla.")
        print("       Aun asi, la red de seguridad real es la supervision en vivo")
        print("       de daq.py: nunca se lanza la secuencia a ciegas.")
    else:
        print("    [ok] la secuencia se mantiene dentro de la ventana")

    res = {"t": sec["t"], "posicion": x, "v": v}

    # --- completo (opcional, sobre un trozo) ------------------------------
    if completo:
        n_trozo = min(len(u), int(20.0 / Ts))
        dec = max(1, int(round(0.002 / Ts)))
        print(f"\n  VALIDACION FISICA COMPLETA sobre los primeros "
              f"{n_trozo*Ts:.0f} s (dos camaras)")
        r = PlantaHidraulica(p, x0=sec["x0"] * 1e-3).simula(
            u[:n_trozo:dec], Ts * dec)
        print(f"    posicion  : {r['posicion'].min():7.2f} .. {r['posicion'].max():7.2f} mm")
        print(f"    velocidad : {r['v'].min():7.3f} .. {r['v'].max():7.3f} mm/s")
        print(f"    P_A       : {r['presion_A'].min():7.2f} .. {r['presion_A'].max():7.2f} bar")
        print(f"    P_B       : {r['presion_B'].min():7.2f} .. {r['presion_B'].max():7.2f} bar")
        res["fisico"] = r
    return res


# ============================================================================
# 4. SALIDA
# ============================================================================
def guarda_csv(sec: dict, ruta: str) -> None:
    os.makedirs(os.path.dirname(ruta) or ".", exist_ok=True)
    with open(ruta, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "u_V"])
        for t, u in zip(sec["t"], sec["u"]):
            w.writerow([f"{t:.6f}", f"{u:.6f}"])
    print(f"\n[ok] {ruta}  ({len(sec['u'])} muestras)")


def grafica(sec: dict, r: dict | None, ruta: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    nfil = 3 if r is not None else 2
    fig, ax = plt.subplots(nfil, 1, figsize=(11, 3 * nfil), sharex=False)

    ax[0].plot(sec["t"], sec["u"], lw=0.5, color="tab:orange")
    ax[0].set_ylabel("comando [V]")
    ax[0].set_xlabel("t [s]")
    ax[0].set_title(f"Secuencia de excitacion (semilla {sec['seed']})")
    ax[0].grid(alpha=0.3)

    au = np.abs(sec["u"])
    au = au[au > 0]
    ax[1].hist(np.log10(au), bins=60, color="tab:blue")
    ax[1].set_xlabel("log10 |u| [V]")
    ax[1].set_ylabel("muestras")
    ax[1].set_title("Reparto de amplitudes (logaritmico: denso donde se ensaya)")
    ax[1].grid(alpha=0.3)

    if r is not None:
        x_lo, x_hi = sec["ventana"]
        ax[2].plot(r["t"], r["posicion"], lw=0.6, color="tab:cyan")
        ax[2].axhline(x_lo, color="tab:red", ls="--", lw=0.8)
        ax[2].axhline(x_hi, color="tab:red", ls="--", lw=0.8)
        ax[2].set_ylabel("posicion [mm]")
        ax[2].set_xlabel("t [s]")
        ax[2].set_title("Recorrido simulado y ventana de seguridad")
        ax[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(ruta, dpi=110)
    print(f"[ok] {ruta}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Disenia la secuencia de excitacion.")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--etiqueta", default="train", help="train | val | ...")
    ap.add_argument("--dur", type=float, default=600.0, help="duracion [s]")
    ap.add_argument("--Ts", type=float, default=0.001, help="periodo del AO [s]")
    ap.add_argument("--u-null", type=float, default=U_NULL,
                    help="comando de velocidad cero [V] (medido: -0.370)")
    ap.add_argument("--umax", type=float, default=10.0)
    ap.add_argument("--x0", type=float, default=75.0, help="posicion inicial [mm]")
    ap.add_argument("--xlo", type=float, default=25.0, help="limite inferior [mm]")
    ap.add_argument("--xhi", type=float, default=125.0, help="limite superior [mm]")
    ap.add_argument("--dx", type=float, default=8.0, help="recorrido por tramo [mm]")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--sin-sim", action="store_true", help="no validar con el simulador")
    ap.add_argument("--usar-modelo", action="store_true",
                    help="validar con las ganancias del modelo en vez de las medidas")
    ap.add_argument("--sim-completo", action="store_true",
                    help="ademas, fisica de dos camaras sobre un trozo")
    args = ap.parse_args()

    sec = disenia_secuencia(
        dur_total=args.dur, Ts=args.Ts, u_max=args.umax,
        x0=args.x0, x_lo=args.xlo, x_hi=args.xhi, dx_obj=args.dx,
        u_null=args.u_null, seed=args.seed,
    )
    informe(sec)
    r = (None if args.sin_sim else
         valida_con_simulador(sec, completo=args.sim_completo,
                              usar_medido=not args.usar_modelo))
    guarda_csv(sec, f"results/excitacion_{args.etiqueta}.csv")
    if args.plot:
        grafica(sec, r, f"results/excitacion_{args.etiqueta}.png")


if __name__ == "__main__":
    main()
