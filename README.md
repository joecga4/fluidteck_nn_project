# Identificación y control neuronal de una prensa servohidráulica

Modelado con redes neuronales e implementación de un neurocontrolador sobre la
**UPH 50** (Fluidtek), la prensa servohidráulica de ensayo de materiales del
laboratorio de la PUCP.

El objetivo es sustituir/complementar el PID con autotuning que gobierna hoy el
equipo por un control basado en redes, en la estructura clásica de dos fases:
**identificación** (una red que modela la dinámica del servo-sistema, validada
en simulación libre) y **control** (un neurocontrolador entrenado contra ese
modelo y comparado con el PID en la máquina real).

> La documentación completa —hardware, modelo físico, resultados medidos,
> metodología y decisiones— está en **[`CLAUDE.md`](CLAUDE.md)**, que hace
> también de fuente para el informe. El protocolo de la sesión de máquina está
> en [`docs/protocolo_fase0.md`](docs/protocolo_fase0.md).

---

## El problema, en una tabla

Todo lo que sigue está **medido en el equipo**, no supuesto:

| | |
|---|---|
| Ganancia comando→velocidad | `v = 0.4459·u + 0.1139` (u>0) · `v = 0.3779·u + 0.1397` (u<0) |
| Asimetría medida | **0.847** (el modelo de dos cámaras predice 0.772) |
| Deriva a comando cero | **+8.4 mm/min** — el peso cae por la fuga del null |
| Comando de velocidad nula | **≈ −0.37 V**, no 0 V |
| Velocidades normadas | 1.5 mm/min (losa) → −0.304 V · 0.1 mm/min (viga) → −0.365 V |

Las dos consignas de ensayo **se separan 62 mV** y viven pegadas al cruce por
cero, donde además cambia la rama de ganancia. Un PID lineal tiene que gobernar
ahí con la misma sintonía que a 1 mm/s. Ese es el hueco que justifica la red.

---

## Estructura

```
CLAUDE.md                  documentación maestra (hardware, modelo, resultados)
docs/protocolo_fase0.md    checklist de la sesión de máquina
tools/
  daq.py                   capa de E/S sobre el cDAQ-9184 (nidaqmx)
  planta_sim.py            simulador físico de dos cámaras
  gen_excitacion.py        diseño de la secuencia de identificación
results/                   datos medidos y figuras
```

## Requisitos

```
python -m pip install numpy matplotlib nidaqmx
```

`nidaqmx` necesita además el driver **NI-DAQmx** instalado (probado con 26.0) y
un chasis **NI cDAQ-9184** con los módulos 9222 (AI), 9263 (AO), 9421 (DI) y
9472 (DO).

## Uso

Sin hardware — todo esto corre en cualquier PC:

```bash
python tools/planta_sim.py --check              # parámetros del modelo
python tools/planta_sim.py --demo asimetria     # extensión vs retracción
python tools/gen_excitacion.py --seed 1 --etiqueta train --plot
```

Con el equipo. **Nada escribe en la salida analógica sin `--armar`**:

```bash
python tools/daq.py --diag                      # solo lectura
python tools/daq.py --di                        # enclavamientos
python tools/daq.py --caracteriza --con-hpu --armar
python tools/daq.py --calibra-presion --secs 180 --previa 60 --armar
```

## ⚠ Seguridad

Este código mueve un cilindro hidráulico capaz de **200 kN a 100 bar**, sobre
una planta con amortiguamiento prácticamente nulo. `tools/daq.py` lleva límites
de posición, fuerza y presión por software, exige `--armar` para cualquier
escritura y devuelve la salida a un estado seguro al terminar o al abortar.

**Nada de eso sustituye a los enclavamientos físicos** —seta de emergencia,
limitadora de presión, finales de carrera—, que son la última línea de defensa y
deben estar operativos. No ejecutar ninguna maniobra sin alguien presente en la
máquina.

## Lo que no está en el repositorio

- **`labview/`** — material original del laboratorio (195 MB). Lo que el
  proyecto necesitaba de esos datos está extraído y analizado en `CLAUDE.md §5.1`.
- **La memoria de Fluidtek (2017)** — documento de ingeniería de un tercero.
  Sus parámetros relevantes están recogidos en `CLAUDE.md §2` y `§3`, junto con
  las discrepancias encontradas al contrastarlos con el equipo real.

## Proyecto hermano

[`pi5_qnx_project`](../pi5_qnx_project) — misma metodología (excitación rica →
NARX → neurocontrolador → comparación de leyes en la planta real) aplicada a un
motor DC bajo QNX. Allí la ley mixta PID+red redujo el sobreimpulso 6–8× con
error de régimen cero; es la hipótesis de partida aquí.
