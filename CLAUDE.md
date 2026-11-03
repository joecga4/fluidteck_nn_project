# CLAUDE.md — Prensa servohidráulica Fluidtek UPH 50 (LEDI-PUCP)

> Documento maestro del proyecto: hardware, modelo, resultados vigentes, estado y
> pendientes. Reemplaza a `PENDIENTES.md` y `docs/protocolo_fase0.md` (eliminados el
> 2026-11-02). El detalle histórico de las mediciones de agosto 2026 (procedimientos,
> datos crudos, NARX/DBP) está en `.claude/skills/resultados-medidos/SKILL.md`; ahí los
> números que dependen de la geometría del cilindro usan la de la memoria (160/100 mm),
> hoy en revisión (§2.3).
>
> **Orden de autoridad de los datos:** medido en el equipo > placa/catálogo > memoria de
> Fluidtek 2017 (`docs/`, desactualizada).

---

## 1. Objetivo

Prensa de ensayo de materiales (losas, vigas de concreto) del Laboratorio de Estructuras
PUCP. Hoy se controla con LabVIEW (PID de posición) sobre un NI cDAQ-9184 por Ethernet.

Líneas de trabajo:
1. **Software de control LabVIEW** (`labview_control_software/`): v15 en uso.
2. **Modelo físico en Simulink** (`simulink/`), Jelali & Kroll 2003.
3. **Identificación y control neuronal** (NARX → neurocontrolador → comparación con el PID).
4. **Diagrama eléctrico** del tablero en CADe_SIMU (`diagrama_electrico/`).

Modos de ensayo: posición (aproximación), fuerza (precarga), rampa de fuerza (kN/min) y
velocidad (losa 1.5 mm/min, viga 0.1 mm/min).

---

## 2. Hardware

### 2.1 Unidad hidráulica (placa UPH 50, Fluidtek 02/2015)
| | |
|---|---|
| Tanque | 50 L, indicador de nivel/temperatura HL91 |
| Bomba | engranajes Casappa PLP10-1, **1.7 L/min**; Pmáx 150 bar, Ptrab **100 bar** |
| Motor M1 | WEG W22, **1.5 kW (2 HP)**, 1730 rpm, 12 bornes; conectado **ΔΔ a 220 V** (6.08 A) |
| Circuito | filtro de succión y de retorno con bypass, limitadora, antirretorno, manómetro, acumulador de vejiga (volumen y precarga **desconocidos**), filtros en línea con señal de saturación |
| Aceite | RANDO HD |

La memoria calculó 1 HP; el motor instalado es de 2 HP. La placa dice 380 V, pero los
guardamotores (6–10 A, ajustados a ~6.2 A) solo cuadran con 220 V.

### 2.2 Servoválvula Moog G761-3001B H04JOFM4VPL
4 L/min a 35 bar por arista · carrete **zero lap** · ±40 mA (paralelo) · **−3 dB a 120 Hz**
(no los 150 Hz de la memoria) · histéresis ≤ 3 % · deriva de null ≤ 2 % por 55 °C
(≈ 0.20 V de comando: **el null deriva con la temperatura del aceite**).
Mando: NI 9263 ±10 V → convertidor Delta VC2124 → **±40 mA** (`K_amp = 0.004 A/V`).

### 2.3 Actuador — ⚠ geometría en revisión
| | Memoria | Medido / provisional |
|---|---|---|
| Émbolo D | 160 mm | **125 mm (provisional, inferido)**; tubo exterior **190 mm (medido)** |
| Vástago d | 100 mm | **80 mm (medido 2026-10-31)** |
| A_A / A_B | 201.1 / 122.5 cm² | **122.7 / 72.5 cm²** |
| Carrera | — | **física 300 mm**; útil 0–150 mm (límite de la VI y del Temposonic) |
| Montaje | vertical, vástago hacia abajo; masa móvil 150 kg + celda 28 kg | |

**Por qué 125 mm (inferido, no medido).** El vástago de 80 mm es medido, así que el 160/100
de la memoria queda descartado. Tres evidencias independientes apuntan a D ≈ 125–128 mm:
1. Relación de presiones en equilibrio P_B/P_A = A_A/A_B = 50/30 = 1.667 (1.64–1.70 en la
   prueba de saturación) → con d = 80 mm, D = 124.7–128.1 mm. Con 150 o 160 mm la relación
   sería 1.40 o 1.33.
2. Bomba de 1.7 L/min (placa) y 2.64 mm/s sostenidos al extender → A ≈ 107–123 cm².
3. El tubo de 190 mm de exterior deja 32.5 mm de pared con D = 125 mm: gruesa para 100 bar
   pero no contradictoria (≈ 38 MPa de tensión en el interior a 150 bar), y coherente con
   un cilindro pensado para la clase de 200 kN, que con 125 mm exige ~163 bar. Con 160 mm
   serían 15 mm, más habitual: este dato por sí solo no decide.

**Pendiente: confirmar D** (placa del cilindro o prueba de fuerza contra la celda:
F_celda / P_A = A_A → ~37 kN a 30 bar si es 125 mm, ~60 kN si es 160 mm; prueba prevista
el 2026-11-03). Fuerza hidráulica: `F = P_A·A_A − P_B·A_B` (cilindro asimétrico).

**Episodio de sobrecarga (informado por el laboratorio, 2026-11-02).** El equipo llegó
alguna vez a ~170–180 kN, empezó a saturar y **falló una soldadura del manifold**. Con
125 mm eso son 139–147 bar (1.4–1.5× los 100 bar de trabajo, cerca de los 150 bar de la
bomba); con 160 mm habrían bastado 85–90 bar, cosa que la geometría descarta. Consecuencias:
a 100 bar la fuerza máxima es **123 kN**; el límite de 170 kN de `PID.c` (v11) viene de ese
episodio y **no es un margen seguro**. Fecha, presiones registradas y tarado real de la
limitadora: desconocidos (§5).

### 2.4 Instrumentación y E/S (cDAQ-9184, alias `cDAQ9184-1ADC24C`)
| Canal | Señal | Escala |
|---|---|---|
| `ai0` (NI 9222) | posición, Temposonic | 0–10 V → 0–150 mm (15 mm/V) |
| `ai1` | celda de carga (convertidor multirango) | 0–10 V → 0–200 kN (20 kN/V) |
| `ai2` | presión cámara A | 4–20 mA / 500 Ω: **32.293 bar/V**, cero 1.9697 V |
| `ai3` | presión cámara B | **18.163 bar/V**, cero 1.9801 V |
| `ao0` (NI 9263) | comando servoválvula | ±10 V → ±40 mA |

Escalas de presión calibradas por dos puntos contra manómetros (2026-08-12). Los fondos
que resultan (259 / 146 bar) no coinciden con la placa reportada (100 / 400 bar): pendiente.

**Digitales — confirmadas por etiquetas de cableado, memoria y VI (2026-10-31).**
Cada `DOn` del NI 9472 activa el relé `K(n+1)` (Phoenix 2961192, 24 VDC).

| Línea | Función | Elemento |
|---|---|---|
| DI0 | seta de emergencia | contacto NC 21-22 · 1 = seguro |
| DI1 | nivel de aceite | 1 = OK |
| DI2 | saturación del filtro en línea | 1 = limpio |
| DI5 | guardamotor GM1 | aux NC 21-22 · **1 = disparado o apagado** |
| DI6 | guardamotor GM2 | aux NC 21-22 |
| DI7 | confirmación de marcha | KM1 aux 13-14 · 1 = motor activo |
| DO0 | arranque bomba | K1 → bobina KM1 → M1 |
| DO1 | circuito M2 (reserva, no hay segundo motor en la UPH) | K2 → KV1 |
| DO5 | **alimentación de los transductores de presión** | K6 (bornes 77/78); alto siempre |
| DO6 | alimenta algo de la hoja 10 (borne 82) | K7 · sin identificar |
| DO2–4, DO7 | sin identificar (K3–K5, K8) | |

⚠ **La seta solo llega a DI0**: K1 alimenta directo la bobina de KM1, así que la parada de
emergencia depende de que la VI lea DI0 y baje DO0. Verificar con la VI detenida y, si no
corta, cablear la seta en serie con el mando de KM1 (electricista).

### 2.5 Tablero
QA1 (Siemens 3VT1, 32 A) → repartidor RB → GM1 (GV2ME14) → KM1 (LC1D09BL, bobina 24 VDC)
→ M1 · GM2 → KV1 → (M2, reserva) · CB1–CB3 (230 V) · PS1 Allen-Bradley 24 VDC 5 A ·
CB4–CB6 (24 VDC) · fusible F3 → Vsup del NI 9472. El cDAQ solo está disponible después de
encender QA1 (la v15 reintenta la reserva al arrancar).

---

## 3. Modelo Simulink (`simulink/`)

Jelali & Kroll 2003: servoválvula de 2.º orden, orificios con underlap y offset de null
(ec. 4.5–4.6), cámaras con E'(p) de Lee y volúmenes variables (4.51–4.52), suministro
dinámico con bomba + limitadora + acumulador (4.2.4), fricción de Stribeck (4.60), peso,
topes y sensores con escala, ruido, ADC y latencia.

- `params_prensa.m`: todos los parámetros, cada uno con su origen (MEDIDO / PLACA /
  CATÁLOGO / AJUSTADO / SUPUESTO). Vigente: **125/80 mm, Qp 1.7 L/min, kQ 0.505,
  x0 1.98 %, xu 1.11 %, L_fis 0.300 m, Ps_ini**.
- `prensa_fluidtek.slx`: generado por `construir_modelo_prensa.m`; validación con
  `validar_modelo_prensa.m` y `sim_satura.m` (estos scripts y los resultados están fuera
  del repositorio, en el respaldo de `D:\fluidteck_project`).

**Validación (2026-10-31):** curva comando→velocidad RMS **0.039 mm/s** (08-12) y
**0.108 mm/s** (08-21); presiones en reposo modelo 28/49 bar vs manómetros 30/50; prueba
de saturación ±10 V: error de posición 2.4 / 1.9 mm en ~98 mm.

**Prueba de saturación (2026-10-31, `daq.py --satura`):** a ±10 V la presión de
suministro cae a ~47 bar; extendiendo la velocidad decae de 4.0 a 2.64 mm/s (se vacía el
acumulador), retrayendo sube de 2.53 a 2.73 mm/s. Los 3.92 mm/s de la curva del 08-21
los aportaba el acumulador; la bomba es la de la placa.

---

## 4. Resultados vigentes (detalle en la skill `resultados-medidos`)

- **Comando → velocidad** (sin probeta): `u>0: v = 0.4459·u + 0.1139` ·
  `u<0: v = 0.3779·u + 0.1397` mm/s. Sin zona muerta (zero lap).
- **Null ≈ −0.37 V**; a 0 V el vástago cae 8.4 mm/min (peso + fuga). El null se movió
  71 mV en 10 min con la temperatura: **el lazo necesita acción integral** y las
  comparaciones deben hacerse en la misma sesión, anotando la temperatura del aceite.
- Consignas normadas: losa 1.5 mm/min → −0.304 V; viga 0.1 mm/min → −0.365 V
  (a 4.6 mV del cruce por cero).
- **Esperar 3 min tras arrancar la bomba** antes de medir (transitorio de presión).
- **Latencia del lazo** cDAQ por Ethernet: 5.46 ms mediana (máx 9.9 ms) → **Ts de
  control 20 ms**. Si sale ~200 ms, la tarea AI no se arrancó explícitamente.
- **Ruido** (σ): posición 0.105 mm, fuerza 0.42 kN, PA 0.07 bar, PB 0.29 bar; modo común
  a 133.8 Hz en los sensores de 24 V. El ruido de posición es el techo de los modelos.
- **Modelos NARX (Fase 1):** estático FIR 50.1 % en simulación libre (87 % del techo que
  impone el ruido); dinámico DBP 48.4 %. El sesgo de validación es la deriva del null.

---

## 5. Estado y pendientes (al 2026-11-02)

| Tema | Estado |
|---|---|
| VI v15 (reserva automática del cDAQ, protección NaN) | en uso; falta prueba tras reinicio completo con UPH encendida |
| Licencia driver HBM LabVIEW (QuantumX MX840B) | vencía ~2026-11-05; cotización pedida a HBK el 2026-10-28; si vence, registrar en catman (el control por cDAQ no depende de ella) |
| Diagrama eléctrico CADe_SIMU | hecho y simulado con las E/S reales (§2.4) |
| Modelo Simulink | validado con 125/80 provisional |
| Diámetro interior del cilindro | **pendiente** — define áreas, fuerza y caudal; prueba F_celda / P_A prevista el 2026-11-03 |
| Episodio de 170–180 kN (soldadura del manifold) | recuperar fecha y presiones (`fuerzavscarga18.xls` de 2017, en el respaldo) y verificar el estado y la presión de diseño del manifold antes de presurizar cerca de 100 bar |
| `PID.c` (Formula Node de la v15) | reescrito para legibilidad (equivalente al original, verificado con un traductor C→Python), con presiones calibradas (§2.4) y `Fp` con 125/80. **Falta pegarlo en la VI** (detenida, UPH apagada) y comprobar con la UPH en marcha que `Lg[12]`/`Lg[13]` marcan ≈ 50/30 bar a −0.37 V. Decisión pendiente: bajar `Fcorte` (160 kN; ~110–115 kN a 100 bar) y añadir corte por presión de la cámara B |
| Manómetro de la UPH | leer en reposo (tarado real de la limitadora) y durante un recorrido a ±10 V; el modelo da Ps ≈ 100 bar al retraer y la medida ~47 bar |
| Acumulador | buscar placa (volumen, precarga) |
| Escalas de presión | revisar placa de los transductores |
| `tools/planta.py` (fuera del repo) | sigue con 160/100: actualizar cuando se confirme D |
| Fricción de Stribeck, volúmenes muertos | supuestos: identificar con rampas lentas a velocidad constante |
| Seta de emergencia | verificar si corta KM1 sin la VI (§2.4) |
| Neurocontrolador (BPTT) y comparación con el PID | sin empezar |
| Harness de lazo cerrado en Simulink con el controlador de la v15 | sin empezar |
| Agentic Toolkit MATLAB/Simulink | instalado en la PC con MATLAB; falta `satk_initialize` + `shareMATLABSession()` en `startup.m` |

---

## 6. Seguridad (no negociable)

Cilindro de hasta 200 kN, planta casi sin amortiguamiento (δ_h ≈ 1e-3).
- Alguien junto a la seta en toda maniobra; sin probeta para identificación.
- **No presurizar por encima de la presión de trabajo** (100 bar ≈ 123 kN con 125 mm): el
  equipo ya llegó a 170–180 kN y falló una soldadura del manifold (§2.3). El corte de
  160 kN de `PID.c` no es margen seguro mientras no se rebaje.
- AO a 0 V **antes** de energizar la bomba (el NI 9263 conserva la última tensión).
- Límites por software: ventana de trabajo 20–130 mm (recolocación 4–146 mm),
  fuerza 50 kN, PA 110 bar, PB 130 bar.
- LabVIEW y NI MAX reservan los módulos: cerrarlos antes de usar Python (y viceversa).
- Editar VIs solo con la VI detenida y la UPH apagada. En HBM `Init.vi`, siempre *Skip*.

---

## 7. Inventario

| Ruta | Contenido |
|---|---|
| `CLAUDE.md` | este documento |
| `README.md` | portada breve |
| `.claude/skills/resultados-medidos/SKILL.md` | registro detallado de las mediciones de agosto 2026 (material para el informe) |
| `labview_control_software/` | VIs v13 (obsoleta: lee fuerza del QuantumX) y **v15** (producción), `GlobalesPrensa.vi`, `TIME.vi`, `PID.c` (código del Formula Node del controlador, para pegar en la VI) |
| `simulink/` | `params_prensa.m`, `prensa_fluidtek.slx` |
| `matlab/Prensa.m` | script MATLAB del proyecto |
| `diagrama_electrico/` | `FLUIDTEK_UPH50.cad` (CADe_SIMU V4.2) y su generador `fluidtek.py` |
| `docs/` | memoria Fluidtek 2017, Jelali & Kroll 2003 cap. 2–4, catálogo Moog 761, EFT (no versionados) |
| `results/` | vacío en este equipo; datos medidos en el respaldo |
| Copias fuera del repo | `C:\Users\LEDI\fluidteck_respaldo_simulink\` (scripts de Simulink retirados) y `C:\Users\LEDI\fluidteck_respaldo_labview\PID_v15_original.c` (Formula Node original) |

---

## 8. Convenciones

- Responder en **español**. Unidades: mm, mm/s, kN, bar, V.
- Signo: u > 0 extiende el vástago (hacia abajo) y aumenta la posición medida.
- Redes neuronales con numpy a mano (despliegue = pesos dentro de un VI); NARX sobre
  **incrementos**, métrica = **simulación libre**.
- Antes de usar un número de la memoria de 2017, contrastarlo con este documento.
