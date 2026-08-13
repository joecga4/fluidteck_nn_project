#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
nn_modelo.py — Modelo NARX de la prensa servohidraulica (Fase 1).

============================ GUIA DE LECTURA ============================
Que hace
--------
Identifica la dinamica comando -> movimiento con una red neuronal entrenada
sobre las capturas del equipo (`results/captura_{train,val}.csv`), y la valida
en SIMULACION LIBRE, que es el uso real del modelo.

LA DECISION ESTRUCTURAL: se predice el INCREMENTO, no la posicion
------------------------------------------------------------------
La planta es un INTEGRADOR (CLAUDE.md §3.3): el comando fija la velocidad y la
posicion es su integral. Si la red predijera la posicion absoluta,

        y(k+1) = f( y(k), y(k-1), ..., u(k), ... )

el termino y(k+1) ~ y(k) domina por completo y la red aprende la identidad: el
ajuste sale del 99 % y el modelo no sirve para nada. La formulacion correcta es

        dy(k+1) = f( dy(k), ..., dy(k-ny+1), u(k), ..., u(k-nu+1) )
        y(k+1)  = y(k) + dy(k+1)

donde dy = y(k) - y(k-1). En simulacion libre se realimenta el INCREMENTO
predicho y se reintegra.

EL Ts DEL MODELO NO ES EL Ts DEL CONTROL, y hay una razon medida
-----------------------------------------------------------------
El ruido de posicion es sigma = 0.105 mm (§5.2). Al diezmar promediando N
muestras baja como sqrt(N), pero el incremento por muestra CRECE con Ts. La
relacion senial/ruido del incremento a 1 mm/s -que es el orden del ensayo de
losa- sale asi:

        Ts        sigma_y     |dy| a 1 mm/s     SNR
        20 ms     0.0235 mm     0.020 mm        0.60     <- bajo el ruido
        50 ms     0.0148 mm     0.050 mm        2.38
       100 ms     0.0105 mm     0.100 mm        6.73     <- por defecto
       200 ms     0.0074 mm     0.200 mm       19.05

A Ts = 20 ms (el del lazo de control, §6.1) el incremento esta POR DEBAJO del
ruido justo en la banda que importa. Por eso el modelo se identifica mas lento.

No se pierde dinamica al hacerlo: a 50 Hz la servovalvula (120 Hz) y la
resonancia hidraulica (>315 Hz) ya estan por encima de Nyquist. En esta banda
la planta es un integrador con una ganancia estatica no lineal MAS un retardo
de transporte de ~5.5 ms; no hay dinamica rapida que capturar. Bajar el Ts solo
anadiria ruido.

RESULTADO PRINCIPAL: EL MEJOR MODELO NO TIENE REALIMENTACION
-------------------------------------------------------------
Se barrio cuanta historia de dy conviene realimentar, y el resultado va en
contra de la intuicion NARX (Ts = 100 ms, nu = 3, 3 reinicios):

    ny    un paso    LIBRE train    LIBRE val
     0     51.5 %      51.5 %         49.6 %     <-- el mejor
     1     56.1 %    -133.5 %       -440.9 %     <-- DIVERGE
     2     62.2 %      37.8 %         41.8 %
     3     62.1 %      33.9 %         31.9 %
     5     63.0 %      44.1 %         41.8 %

Cada dy realimentado MEJORA el ajuste a un paso y EMPEORA la simulacion libre.
La razon es medible: dy lleva el ruido del sensor de posicion (SNR ~ 6 a
Ts = 100 ms, §5.2) y u no lleva ninguno. Al entrenar a un paso, la red descubre
que puede usar el ruido de dy(k) para predecir parte del ruido de dy(k+1) —eso
sube el ajuste a un paso— pero en simulacion libre esa entrada ya no es el
ruido medido sino el ERROR de la propia red, que se realimenta y crece.

Y encaja con la fisica: a 100 ms la servovalvula (120 Hz) y la resonancia
hidraulica (>315 Hz) estan MUY por encima de Nyquist. En esta banda la planta
es un integrador con ganancia estatica no lineal, sin estado interno propio que
recordar. No hay nada que realimentar.

Por eso el modelo por defecto es FIR NO LINEAL:

        dy(k+1) = f( u(k), u(k-1) )

que ademas no puede acumular error por construccion: la simulacion libre y el
un paso son la MISMA cuenta. Barrido de nu (ny=0): 48.7 % con nu=1, 50.1 % con
nu=2, y a partir de nu=5 el ajuste de train sube mientras el de val baja.

METRICA: SIMULACION LIBRE, nunca "un paso adelante"
----------------------------------------------------
El "un paso" recibe la salida MEDIDA en cada instante y siempre se ve bien:
en una planta integradora es practicamente un test de que la red sabe copiar.
La simulacion libre realimenta la prediccion de la red, que es como se usa el
modelo cuando el neurocontrolador se entrena contra el. Es la unica que informa.

Para una planta integradora hay que medirla sobre el INCREMENTO (la velocidad):
la posicion es su integral y cualquier sesgo, por pequenio que sea, se acumula
sin limite. La reconstruccion de posicion se reporta aparte, por VENTANAS.

⚠ LINEA BASE, no modelo final
------------------------------
Esta version IGNORA a proposito que el null se desplazo 71 mV entre train y val
(CLAUDE.md §5.6c) — unos 20 C de aceite. Es la opcion (1) de las tres que se
plantearon: sirve de referencia y hace VISIBLE el problema. Se espera un SESGO
en validacion que ninguna cantidad de neuronas va a arreglar, porque no es
sobreajuste: es pedirle a un modelo que describa dos plantas ligeramente
distintas. El programa lo mide y lo reporta explicitamente.

Uso
---
    python tools/nn_modelo.py                      # FIR, Ts=100 ms, 15 ocultas
    python tools/nn_modelo.py --Ts 0.05 --hidden 30
    python tools/nn_modelo.py --barrido-ts         # compara varios Ts
"""

from __future__ import annotations

import argparse
import os

import numpy as np


# ============================================================================
# 1. DATOS
# ============================================================================
def carga(ruta: str, Ts: float, fs_orig: float = 1000.0) -> dict:
    """Lee una captura y la diezma a `Ts` PROMEDIANDO.

    El promediado no es un detalle: hace de filtro antialiasing y ademas baja
    el ruido como sqrt(N). Quedarse con una muestra de cada N (diezmado a secas)
    dejaria pasar todo el ruido de banda ancha y los tonos de 133.8 Hz y
    1462.6 Hz que se midieron en §5.2.
    """
    d = np.genfromtxt(ruta, delimiter=",", names=True)
    n_dec = int(round(Ts * fs_orig))
    n = (len(d["t_s"]) // n_dec) * n_dec

    def prom(v):
        return v[:n].reshape(-1, n_dec).mean(axis=1)

    return {
        "t": prom(d["t_s"]),
        "u": prom(d["u_V"]),
        "y": prom(d["posicion_mm"]),
        "P_A": prom(d["presion_A_bar"]),
        "P_B": prom(d["presion_B_bar"]),
        "Ts": Ts,
        "ruta": ruta,
    }


def regresor(dat: dict, ny: int, nu: int) -> tuple:
    """Construye la matriz de regresores y el objetivo.

        entrada X[k] = [ dy(k), ..., dy(k-ny+1), u(k), ..., u(k-nu+1) ]
        objetivo d[k] = dy(k+1)

    Devuelve (X, d, k0) donde k0 es el indice de la muestra original que
    corresponde a la primera fila, necesario para la simulacion libre.
    """
    y, u = dat["y"], dat["u"]
    dy = np.diff(y)                       # dy[i] = y[i+1] - y[i]
    k0 = max(ny, nu)                      # primera k con historia completa
    N = len(dy) - k0 - 1
    X = np.empty((N, ny + nu))
    for j in range(ny):                   # dy(k-j)
        X[:, j] = dy[k0 - j: k0 - j + N]
    for j in range(nu):                   # u(k-j)
        X[:, ny + j] = u[k0 - j: k0 - j + N]
    d = dy[k0 + 1: k0 + 1 + N]            # dy(k+1)
    return X, d, k0


# ============================================================================
# 2. LA RED, escrita a mano
# ============================================================================
class RedNARX:
    """Perceptron de una capa oculta: ne - nh - 1.

        h = tanh(X·V + bv)          [N, nh]
        o = h·W + bw                [N]

    Se escribe el gradiente de forma explicita porque el proyecto es didactico
    y tiene que poder reproducirse en el informe. Con  E = 1/2 <(o - d)^2>:

        e      = o - d                          [N]
        dE/dW  = h^T·e / N
        dE/dbw = <e>
        delta  = (e ⊗ W) * (1 - h^2)            [N, nh]   (derivada de tanh)
        dE/dV  = X^T·delta / N
        dE/dbv = <delta>
    """

    def __init__(self, ne: int, nh: int, rng: np.random.Generator):
        # Inicializacion de Xavier: mantiene la varianza al atravesar la capa
        self.V = rng.normal(0, np.sqrt(1.0 / ne), (ne, nh))
        self.bv = np.zeros(nh)
        self.W = rng.normal(0, np.sqrt(1.0 / nh), nh)
        self.bw = 0.0

    # -------------------------------------------------------------- adelante
    def adelante(self, X):
        h = np.tanh(X @ self.V + self.bv)
        return h @ self.W + self.bw, h

    def __call__(self, X):
        return self.adelante(X)[0]

    # ---------------------------------------------------------------- atras
    def gradiente(self, X, d):
        N = len(d)
        o, h = self.adelante(X)
        e = o - d
        gW = h.T @ e / N
        gbw = e.mean()
        delta = np.outer(e, self.W) * (1.0 - h * h)
        gV = X.T @ delta / N
        gbv = delta.mean(axis=0)
        return (gV, gbv, gW, gbw), 0.5 * np.mean(e * e)

    # ------------------------------------------------------------------ Adam
    def entrena(self, X, d, epocas=300, lote=256, lr=3e-3, rng=None,
                verboso=False):
        m = [np.zeros_like(p) for p in (self.V, self.bv, self.W)] + [0.0]
        v = [np.zeros_like(p) for p in (self.V, self.bv, self.W)] + [0.0]
        b1, b2, eps = 0.9, 0.999, 1e-8
        paso = 0
        N = len(d)
        rng = rng or np.random.default_rng(0)
        for ep in range(epocas):
            orden = rng.permutation(N)
            for i in range(0, N, lote):
                idx = orden[i:i + lote]
                g, _ = self.gradiente(X[idx], d[idx])
                paso += 1
                for k, (p_nom, gk) in enumerate(zip(("V", "bv", "W", "bw"), g)):
                    m[k] = b1 * m[k] + (1 - b1) * gk
                    v[k] = b2 * v[k] + (1 - b2) * gk * gk
                    mhat = m[k] / (1 - b1**paso)
                    vhat = v[k] / (1 - b2**paso)
                    act = lr * mhat / (np.sqrt(vhat) + eps)
                    setattr(self, p_nom, getattr(self, p_nom) - act)
            if verboso and (ep + 1) % 50 == 0:
                _, E = self.gradiente(X, d)
                print(f"      epoca {ep+1:4d}   E = {E:.6e}")


# ============================================================================
# 3. SIMULACION LIBRE — la metrica que manda
# ============================================================================
def simula_libre(red, esc, dat: dict, ny: int, nu: int, k0: int, N: int):
    """Realimenta la PREDICCION de la red, no la medida.

    El estado son los `ny` ultimos incrementos PREDICHOS. El comando `u` es
    conocido (es lo que se emitio). Devuelve el vector de incrementos simulados.
    """
    u = dat["u"]
    dy_med = np.diff(dat["y"])
    hist = list(dy_med[k0 - ny + 1: k0 + 1][::-1])    # dy(k), dy(k-1), ...
    salida = np.empty(N)
    for i in range(N):
        k = k0 + i
        x = np.array(hist[:ny] + [u[k - j] for j in range(nu)])
        xn = (x - esc["mx"]) / esc["sx"]
        dyp = float(red(xn[None, :])[0]) * esc["sd"] + esc["md"]
        salida[i] = dyp
        hist.insert(0, dyp)
        hist.pop()
    return salida


def ajuste(real, pred) -> float:
    """Fit % al estilo de la System Identification Toolbox:
        100 * (1 - ||real-pred|| / ||real-mean(real)||)
    100 = perfecto; 0 = tan bueno como predecir la media."""
    return 100.0 * (1.0 - np.linalg.norm(real - pred) /
                    np.linalg.norm(real - real.mean()))


def r2(real, pred) -> float:
    ss = np.sum((real - pred) ** 2)
    st = np.sum((real - real.mean()) ** 2)
    return 1.0 - ss / st


def evalua(red, esc, dat, ny, nu, etiq):
    """Un paso + simulacion libre + reconstruccion de posicion por ventanas."""
    X, d, k0 = regresor(dat, ny, nu)
    Xn = (X - esc["mx"]) / esc["sx"]

    # --- un paso adelante (recibe la medida en cada instante) --------------
    d1 = red(Xn) * esc["sd"] + esc["md"]
    # --- simulacion libre (se realimenta a si misma) -----------------------
    dl = simula_libre(red, esc, dat, ny, nu, k0, len(d))

    res = {
        "etiq": etiq, "n": len(d),
        "fit_1": ajuste(d, d1), "r2_1": r2(d, d1),
        "fit_L": ajuste(d, dl), "r2_L": r2(d, dl),
        "sesgo": float(np.mean(dl - d)),          # mm por muestra
        "Ts": dat["Ts"], "d": d, "dl": dl, "k0": k0,
    }
    res["sesgo_mm_min"] = res["sesgo"] / dat["Ts"] * 60.0
    return res


def posicion_por_ventanas(dat, res, ventana_s=60.0):
    """Reintegra la velocidad simulada en ventanas, reiniciando de la posicion
    MEDIDA al principio de cada una.

    Por que por ventanas: la posicion es la integral del incremento, asi que un
    sesgo constante crece SIN LIMITE. Reintegrar 600 s seguidos no mide la
    calidad del modelo, mide el sesgo multiplicado por el tiempo. Con ventanas
    se ve lo que de verdad importa para el control: cuanto se desvia el modelo
    en el horizonte en que se va a usar.
    """
    Ts = dat["Ts"]; nv = int(ventana_s / Ts)
    y = dat["y"]; k0 = res["k0"]
    errs = []
    for a in range(0, len(res["dl"]) - nv, nv):
        y0 = y[k0 + a]
        yp = y0 + np.cumsum(res["dl"][a:a + nv])
        ym = y[k0 + a + 1: k0 + a + nv + 1]
        errs.append(np.abs(yp - ym).max())
    return np.array(errs)


# ============================================================================
# 4. PROGRAMA
# ============================================================================
def entrena_y_evalua(Ts, ny, nu, nh, epocas, reinicios, semilla, verboso=True):
    tr = carga("results/captura_train.csv", Ts)
    va = carga("results/captura_val.csv", Ts)
    Xtr, dtr, _ = regresor(tr, ny, nu)

    esc = {"mx": Xtr.mean(0), "sx": Xtr.std(0) + 1e-12,
           "md": dtr.mean(), "sd": dtr.std() + 1e-12}
    Xn = (Xtr - esc["mx"]) / esc["sx"]
    dn = (dtr - esc["md"]) / esc["sd"]

    if verboso:
        print(f"\n  Ts = {Ts*1000:.0f} ms   regresor {ny}x dy + {nu}x u = "
              f"{ny+nu} entradas   red {ny+nu}-{nh}-1")
        print(f"  patrones: train {len(dtr)}   val {len(regresor(va,ny,nu)[1])}")

    # SELECCION POR REINICIOS. Entrenar a un paso NO garantiza que el modelo sea
    # estable al realimentarse: hay semillas que ajustan bien y divergen en
    # simulacion libre. Se entrenan varias y se elige la que mejor SIMULA en
    # datos NO VISTOS.
    mejor, mejor_fit = None, -np.inf
    for r in range(reinicios):
        rng = np.random.default_rng(semilla + r)
        red = RedNARX(ny + nu, nh, rng)
        red.entrena(Xn, dn, epocas=epocas, rng=rng)
        rv = evalua(red, esc, va, ny, nu, "val")
        if verboso:
            print(f"    reinicio {r+1}/{reinicios}: fit libre en val = "
                  f"{rv['fit_L']:6.2f} %")
        if rv["fit_L"] > mejor_fit:
            mejor, mejor_fit = red, rv["fit_L"]
    return mejor, esc, tr, va


def techo_ruido(dat: dict, sigma_1k: float = 0.105, fs_orig: float = 1000.0) -> float:
    """Ajuste MAXIMO alcanzable, impuesto por el ruido del sensor.

    Parte de sigma = 0.105 mm medido en reposo (§5.2). Al diezmar promediando N
    muestras el ruido baja como sqrt(N); el incremento y(k)-y(k-1) combina dos
    muestras independientes, de ahi el sqrt(2):

        sigma_dy = sigma_1k / sqrt(N) * sqrt(2)
        techo    = 100 * (1 - sigma_dy / std(dy))

    Sin esta referencia el fit% no se puede interpretar: un 50 % suena mal hasta
    que se ve que el maximo posible es ~58 %.
    """
    N = int(round(dat["Ts"] * fs_orig))
    sigma_dy = sigma_1k / np.sqrt(N) * np.sqrt(2)
    return 100.0 * (1.0 - sigma_dy / np.diff(dat["y"]).std())


def informe(red, esc, tr, va, ny, nu):
    rt = evalua(red, esc, tr, ny, nu, "train")
    rv = evalua(red, esc, va, ny, nu, "val")
    print("\n  " + "=" * 64)
    print(f"  {'serie':7s} {'un paso':>10s} {'SIM LIBRE':>11s} {'R2 libre':>10s}"
          f" {'sesgo':>14s}")
    for r in (rt, rv):
        print(f"  {r['etiq']:7s} {r['fit_1']:9.2f}% {r['fit_L']:10.2f}%"
              f" {r['r2_L']:10.4f} {r['sesgo_mm_min']:+11.3f} mm/min")
    print("  " + "=" * 64)
    for r, dat in ((rt, tr), (rv, va)):
        e = posicion_por_ventanas(dat, r, 60.0)
        print(f"  {r['etiq']:7s}: error MAXIMO de posicion en ventanas de 60 s: "
              f"mediana {np.median(e):5.2f} mm, peor {e.max():5.2f} mm")
    print()
    for r, dat in ((rt, tr), (rv, va)):
        techo = techo_ruido(dat)
        print(f"  {r['etiq']:7s}: techo impuesto por el ruido = {techo:5.1f} %"
              f"   -> alcanzado el {r['fit_L']/techo*100:.0f} % de lo posible")
    if ny == 0:
        print("\n  (ny=0: el modelo NO se realimenta, asi que 'un paso' y")
        print("   'simulacion libre' son la misma cuenta y no puede acumular error)")
    return rt, rv


def main():
    ap = argparse.ArgumentParser(description="Modelo NARX de la prensa (Fase 1)")
    ap.add_argument("--Ts", type=float, default=0.100, help="Ts del modelo [s]")
    ap.add_argument("--ny", type=int, default=0,
                    help="incrementos pasados realimentados (0 = FIR, el mejor)")
    ap.add_argument("--nu", type=int, default=2, help="comandos pasados")
    ap.add_argument("--hidden", type=int, default=15)
    ap.add_argument("--epocas", type=int, default=250)
    ap.add_argument("--reinicios", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--barrido-ts", action="store_true",
                    help="compara varios Ts y sale")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    print("=" * 70)
    print("MODELO NARX SOBRE INCREMENTOS — linea base (ignora la deriva del null)")
    print("=" * 70)

    if args.barrido_ts:
        print(f"\n{'Ts':>7s} {'fit libre train':>16s} {'fit libre val':>14s}"
              f" {'sesgo val':>15s}")
        for Ts in (0.020, 0.050, 0.100, 0.200):
            red, esc, tr, va = entrena_y_evalua(
                Ts, args.ny, args.nu, args.hidden, args.epocas, 2,
                args.seed, verboso=False)
            rt = evalua(red, esc, tr, args.ny, args.nu, "train")
            rv = evalua(red, esc, va, args.ny, args.nu, "val")
            print(f"{Ts*1000:5.0f}ms {rt['fit_L']:15.2f}% {rv['fit_L']:13.2f}%"
                  f" {rv['sesgo_mm_min']:+12.3f} mm/min")
        return

    red, esc, tr, va = entrena_y_evalua(
        args.Ts, args.ny, args.nu, args.hidden, args.epocas,
        args.reinicios, args.seed)
    rt, rv = informe(red, esc, tr, va, args.ny, args.nu)

    os.makedirs("results", exist_ok=True)
    np.savez("results/nn_modelo.npz", V=red.V, bv=red.bv, W=red.W, bw=red.bw,
             mx=esc["mx"], sx=esc["sx"], md=esc["md"], sd=esc["sd"],
             Ts=args.Ts, ny=args.ny, nu=args.nu)
    print("\n[ok] results/nn_modelo.npz")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(13, 7))
        for a, r, dat in zip(ax, (rt, rv), (tr, va)):
            t = dat["t"][r["k0"] + 1: r["k0"] + 1 + r["n"]]
            v_med = r["d"] / dat["Ts"] * 60
            v_sim = r["dl"] / dat["Ts"] * 60
            a.plot(t, v_med, lw=.5, color="tab:cyan", label="medido")
            a.plot(t, v_sim, lw=.5, color="tab:red", alpha=.8,
                   label="simulacion libre")
            a.set_ylabel("velocidad [mm/min]")
            a.set_title(f"{r['etiq']} — fit libre {r['fit_L']:.1f} %, "
                        f"sesgo {r['sesgo_mm_min']:+.2f} mm/min")
            a.legend(fontsize=8); a.grid(alpha=.3)
        ax[-1].set_xlabel("t [s]")
        fig.tight_layout()
        fig.savefig("results/nn_modelo.png", dpi=110)
        print("[ok] results/nn_modelo.png")


if __name__ == "__main__":
    main()
