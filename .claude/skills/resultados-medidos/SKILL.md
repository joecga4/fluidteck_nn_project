---
name: resultados-medidos
description: Registro detallado de las mediciones de Fase 0 y Fase 1 sobre la prensa servohidráulica UPH 50 — auditoría de los datos de 2017, línea base de sensores y análisis de ruido, primer arranque de la HPU, calibración de las escalas de presión, capturas de identificación y resultados completos de los modelos NARX y DBP. Consultar al escribir el informe, al justificar una decisión de diseño, o al revisar cómo se obtuvo un número concreto.
---

# Resultados medidos — registro detallado

> Este fichero es la **sección 5 completa** del proyecto. Se separó de `CLAUDE.md` para que su
> detalle no ocupe contexto en cada sesión, pero **sigue siendo material directo para el
> informe** y se versiona igual que el resto del repositorio.
>
> El **resumen operativo** —los números que se usan a diario— vive en `CLAUDE.md` §5.0. Aquí
> está el *cómo* se obtuvo cada uno: el procedimiento, los datos crudos, las correcciones
> sobre lecturas preliminares y el razonamiento que las cerró.
>
> Las referencias cruzadas (`§2.3`, `§3.3`, `§6.4`…) apuntan a `CLAUDE.md`.

---


### 5.1 Auditoría de los datos históricos (2017) — `labview/Nueva carpeta/data para identificacion/`
Los archivos `.xls` son en realidad **texto tabulado**, no Excel. Dos familias:

**(a) Lazo abierto** `lazoabierto{1,2,3,4,5}.xls` — columnas `ENTRADA (V)` / `SALIDA (mm)`:

| Archivo | N | Niveles de entrada | **Cambios de escalón** |
|---|---|---|---|
| lazoabierto1 | 2505 | 0, +6, −6, −5 | **3** |
| lazoabierto2 | 2108 | 0, ±12 | **3** |
| lazoabierto3 | 2471 | 0, ±24, −20 | **3** |
| lazoabierto4 | 4373 | 0, ±48, −40 | **3** |
| lazoabierto5 | 4417 | 0, ±48, −40 | **3** |

> **Veredicto: no sirven para entrenar una red.** Cada archivo es *una* subida y *una*
> bajada. Un NARX necesita cientos de transiciones a niveles variados; con 4 segmentos por
> archivo el modelo memoriza esos cuatro puntos. Sirven, eso sí, como **verificación
> cruzada** del modelo físico y como evidencia de dos hechos:

**Hecho 1 — la planta es un integrador, y bastante lineal en ganancia de velocidad:**

| Entrada (u) | Velocidad media | Ganancia (mm/s por unidad) |
|---|---|---|
| +6 | +0.081 mm/s | 0.0135 |
| +24 | +0.373 mm/s | 0.0155 |
| +48 | +0.721 mm/s | 0.0150 |
| −24 | −0.259 mm/s | 0.0108 |
| −48 | −0.578 mm/s | 0.0120 |

**Hecho 2 — hay asimetría y hay deriva de null.** La ganancia en sentido negativo es
**~20 % menor** que en positivo (0.0114 vs 0.0147 medio) — compatible con la asimetría de
áreas de un cilindro con vástago pasante parcial (§2.3) y/o con el ajuste del carrete. Y a
**entrada 0 la posición deriva a +0.03 mm/s**: el carrete no está exactamente en null.
Esa deriva es del **orden del 4 %** de la velocidad a fondo de escala — nada despreciable
en un ensayo de 0.1 mm/min.

⚠ **Dos incertidumbres abiertas en estos datos, a resolver con la captura nueva:**
1. **No hay columna de tiempo.** Las velocidades de arriba asumen **Ts = 0.1 s** (el paso
   que sí aparece en `posibleDATA.lvm`). Si el Ts real fue otro, todas escalan por igual —
   las *relaciones* (linealidad, asimetría, deriva) se mantienen; los valores absolutos no.
2. **La unidad de `ENTRADA` no es voltios.** Los niveles ±6/±12/±24/±48 exceden los ±10 V
   del AO; con `K2 = 0.1 V/%` encajan como **% de la variable de control** (48 % = 4.8 V).
   Bajo esa lectura y con Ts = 0.1 s, la ganancia medida (0.015 mm/s por %) queda **~6.5×
   por debajo** de la que predice el modelo físico (0.974 mm/(s·V) ⇒ 0.0974 mm/s por %).
   **Esa discrepancia de 6.5× está sin explicar** y es el primer objetivo cuantitativo de la
   captura nueva. Candidatos: el Ts supuesto, el ajuste real de `K_amp`, la caída de
   presión de trabajo frente a los 70 bar de la curva, o la limitación de caudal de la bomba.

**(b) Ensayos con carga** `fuerzavscarga18.xls` (25 497 muestras, **con columna de tiempo**,
hasta 18.4 t ≈ 180 kN) y `fuerzavscargaconLCDT.xls` (10 321 muestras, 6 t). `Tiempo(s)`
llega a 4056 s con dt ≈ 0.159 s. Son **ensayos de rotura en lazo cerrado**, no
identificación, pero son la única evidencia disponible de la **relación fuerza–desplazamiento
con probeta** y del comportamiento en el instante de la fractura. Útiles para dimensionar el
lazo de fuerza.

### 5.2 Fase 0 (parcial) — línea base de los sensores, medida el 2026-08-12
Primera medida real sobre el equipo: **5 s a 5 kHz en los 4 AI, sin escribir nada en el AO**
(`tools/daq.py --diag`). La planta estaba en reposo.

| Canal | Media | σ (V) | **σ (ing.)** | pico-pico |
|---|---|---|---|---|
| `ai0` posición | 0.1393 V → **2.09 mm** | 6.98 mV | **0.105 mm** | 0.77 mm |
| `ai1` fuerza | −0.0068 V → −0.14 kN | 21.1 mV | **0.422 kN** | 3.69 kN |
| `ai2` presión A | 1.9686 V → **−0.39 bar** | 5.67 mV | **0.071 bar** | 0.68 bar |
| `ai3` presión B | 1.9802 V → **−0.95 bar** | 5.77 mV | **0.289 bar** | 3.09 bar |

*(escalas corregidas: posición 0–150 mm, presión A 0–100 bar y B 0–400 bar sobre span
2–10 V. La cámara B, al tener 4× de rango, paga 4× de ruido en unidades de ingeniería.)*

**(a) Las escalas de presión quedan determinadas — span 2–10 V.** Con los manómetros a 0 y
la HPU apagada, ambos canales leen ~1.97 V; con span 2–10 V eso da **−0.39 y −0.95 bar**,
cero dentro del error (§2.4). *Corrección respecto a la primera lectura de estos datos: con
la escala 0–10 V que se supuso al principio salían 19.7 y 79.2 bar, y el "acuerdo" que se
anotó entre `A_p·P_L` y la celda era una coincidencia entre dos cantidades que en realidad
valen ambas cero.* La comparación **fuerza-celda vs `P_A·A_A − P_B·A_B`** sigue siendo una
validación cruzada valiosa, pero **hay que hacerla con la planta cargada**, no en reposo.

**(b) Hay ruido de modo común a 133.8 Hz y 1462.6 Hz.** Aparece en **posición y en las dos
presiones** —los tres sensores alimentados a 24 V— y **no** en la celda, que va por su
propio amplificador:

| Canal | Componentes dominantes |
|---|---|
| posición | **133.8 Hz : 5.33 mV** · 7.6 Hz : 3.16 mV · 1462.6 Hz : 3.04 mV |
| presión A | 133.8 Hz : 2.19 mV · 1462.6 Hz : 1.78 mV |
| presión B | 133.8 Hz : 2.24 mV · 1462.6 Hz : 1.77 mV |
| fuerza | *(banda ancha, sin tono dominante)* |

No son 50/60 Hz ni sus armónicos, así que **no es red eléctrica**: el patrón apunta a la
**fuente de 24 V compartida** o a un lazo de masa. Es una pista concreta y accionable —
atacarla mejora directamente la medida de velocidad a baja consigna (punto c). Mientras
tanto, como la captura va temporizada por hardware, **muestrear rápido y diezmar promedia
esos tonos casi gratis**.

**(c) Cuánta velocidad se puede medir — el número que condiciona el lazo lento.**
Con σ_posición = 0.279 mm, la incertidumbre de la velocidad estimada por ajuste lineal
sobre una ventana de duración T (a 100 Hz) es:

| Ventana | σ de la velocidad |
|---|---|
| 1 s | **2.18 mm/min** |
| 3 s | **0.42 mm/min** |
| 10 s | **0.069 mm/min** |
| 30 s | **0.013 mm/min** |
| 60 s | **0.0047 mm/min** |

Contrástese con las consignas normadas: **1.5 mm/min (losa)** y **0.1 mm/min (viga)**.
Conclusión operativa: **la velocidad no se puede obtener por diferencia entre muestras
consecutivas**; hace falta ajuste sobre ventana, y la ventana necesaria es de **~2–3 s para
1.5 mm/min** y de **~10–15 s para 0.1 mm/min** (criterio: σ ≤ 10 % de la consigna). Eso acota el ancho de banda alcanzable del
lazo de velocidad y es un límite **de medida**, no de control: ningún controlador, neuronal
o no, puede regular mejor que lo que puede medir. Bajar el ruido de posición (punto b) es
la palanca más directa que tiene este proyecto.

✅ **Resuelto (2026-08-12):** los ~1.97 V de ambos canales no eran presión sino el **cero
del span 2–10 V**. El laboratorio confirmó que los manómetros marcan 0 en ambas cámaras con
la HPU apagada, lo que fija la escala sin ambigüedad (§2.4).

### 5.3 Fase 0 — primer arranque de la UPH desde Python (2026-08-12)

Primera vez que el equipo se opera desde el pipeline nuevo. Tres resultados.

**(a) Mapeo digital resuelto.** El laboratorio leyó las salidas digitales del VI de LabVIEW:

| línea | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| UPH apagada | F | F | F | F | F | **T** | F | F |
| UPH encendida | **T** | F | F | F | F | **T** | F | F |

- **`DO line0` = arranque de la UPH.** Es la única que cambia.
- **`DO line5` = permisivo, energizado siempre**, también con la bomba parada. La memoria
  dice que el NI 9472 «envía señal de parada de emergencia desde la pantalla»: una salida
  alta en reposo *y* en marcha es exactamente eso, un permisivo en lógica negada. Su función
  exacta no está confirmada, pero **el tratamiento correcto es el mismo: sostenerla**.
  Por eso `daq.py` escribe **el puerto entero** (`PuertoDO`) y nunca una línea suelta: al
  liberar LabVIEW su tarea, las salidas pueden volver a 0 y tirar el permisivo.
- **`DI line7` = motor encendido.** Pasa de 0 a 1 al arrancar, verificado dos veces. Las
  `DI line0` y `line1` siguen altas en ambos estados (contactos normalmente cerrados).

**(b) ✅ CURVA COMANDO → VELOCIDAD, medida completa** (`results/caracterizacion_2026-08-12.csv`).
Barrido de 13 escalones de 2.5 s, UPH en marcha, sin probeta:

| u [V] | v [mm/s] | | u [V] | v [mm/s] |
|---|---|---|---|---|
| −2.00 | −0.6252 | | +0.05 | +0.1456 |
| −1.00 | −0.2240 | | +0.10 | +0.1667 |
| −0.50 | −0.0380 | | +0.25 | +0.2230 |
| −0.25 | +0.0452 | | +0.50 | +0.3203 |
| −0.10 | +0.0906 | | +1.00 | +0.5547 |
| −0.05 | +0.1161 | | +2.00 | +1.0120 |
| **0.00** | **+0.1405** | | | |

Ajuste por ramas, **ambas casi perfectamente lineales**:

```
u > 0 :  v = 0.4459·u + 0.1139     (R² = 0.9991)
u < 0 :  v = 0.3779·u + 0.1397     (R² = 0.9986)
```

- **Ganancia: K₊ = 0.446 mm/s·V, K₋ = 0.378 mm/s·V.**
- **Asimetría |K₋/K₊| = 0.847**, frente al **0.772** que predice el modelo de dos cámaras
  (§5.4). Coinciden dentro del 10 % — el modelo asimétrico está bien planteado.
- **Deriva a comando cero: +0.1405 mm/s = 8.43 mm/min**, confirmada.
- **El comando de velocidad cero no es 0 V sino ≈ −0.26 V (rama +) / −0.37 V (rama −).**

**Y esa deriva NO es un carrete descentrado: es el peso cayendo por la fuga del null.**
Con el actuador **en vertical, positivo a favor de la gravedad** (§2.3), y con la celda de
28 kg colgando, a carrete cerrado queda una fuga residual —el centro cerrado no es estanco—
y el conjunto **desciende solo**. Que la deriva sea positiva (hacia afuera, hacia abajo) es
exactamente lo que predice la gravedad. Un offset mecánico del carrete no tendría por qué
apuntar en el sentido del peso.

⚠ **Consecuencia para el modelo, y es seria: esa deriva DEPENDE DE LA CARGA.** Sin probeta,
el peso cae libre por la fuga. Con probeta en contacto, la reacción cambia el balance y la
deriva será otra (menor, nula, o incluso invertida). **Un modelo identificado sin probeta no
predice el comportamiento con probeta cerca del cruce por cero** — que es justo donde ocurre
el ensayo. Es la misma advertencia de §8 sobre el lazo de fuerza, ahora con una causa
concreta. Habrá que decidir si se captura también con una probeta rígida de referencia.

**Qué significa para los ensayos normados.** Ambos caen en la rama negativa:

| Ensayo | velocidad | comando |
|---|---|---|
| Losa | 1.5 mm/min | **−0.3036 V** |
| Viga | 0.1 mm/min | **−0.3654 V** |
| *(velocidad cero)* | 0 | *−0.370 V* |

Se separan **61.7 mV**, y el de viga queda a **4.6 mV del punto de velocidad nula**. El AO de
16 bits da 0.305 mV de LSB, así que resolución hay (15 LSB); lo que hay es un punto de
trabajo pegado al cruce por cero, **donde además cambia la rama** (la ganancia salta de
0.378 a 0.446 al cruzar). Un PID lineal tiene que gobernar ahí con una ganancia que también
sirva a 1 mm/s: es exactamente el hueco que llena un feedforward no lineal aprendido, y el
equivalente hidráulico de la zona muerta del L298N en `pi5_qnx_project`.

> **⚠ Dos correcciones sobre lo que se anotó antes en esta misma sesión.**
>
> 1. **No hay zona muerta de 0.24 V.** Se estimó con dos puntos y salió
>    `v = 0.718·(u − 0.241)`, de donde se concluyó una zona muerta con los ensayos normados
>    en su borde. **El barrido completo lo desmiente**: las dos ramas son lineales con
>    R² ≈ 0.999 y no aparece meseta. El punto malo era el «tanteo» de +0.30 V, tomado
>    **justo tras arrancar la bomba**, cuando las cámaras aún se estaban presurizando
>    (`P_B` sube de 105 a 147 bar durante los primeros escalones). Ese punto daba
>    0.043 mm/s donde el barrido da 0.223 mm/s a +0.25 V. **Lección: no medir nada en los
>    primeros segundos tras arrancar la UPH**; `_descubre_sentido` sigue usándose solo para
>    el signo, que sí acierta.
> 2. **La deriva a comando cero era buena.** Se midió +0.139 mm/s, luego se retiró por
>    considerarla contaminada por ese mismo transitorio, y el barrido la confirma en
>    **+0.1405 mm/s**. Queda como dato válido.
>
> Lo que sí sobrevive de la primera lectura es la conclusión de fondo: **el punto de trabajo
> de los ensayos está en una región estrecha y no lineal, lejos del cero de comando.**

**(b2) Un lazo P puro no puede posicionar esta planta.** Tres barridos consecutivos
abortaron en la recolocación previa, y el motivo es estructural, no de sintonía: con
`Kp = 0.08 V/mm` un error de 0.5 mm pide 0.04 V, y con el null en −0.3 V ese comando ni
siquiera invierte el sentido de la deriva. El vástago **sube** mientras el lazo pide bajar.
Se resolvió no exigiendo posición exacta antes de barrer (basta ±25 mm del centro) y
añadiendo compensación de offset al comando. Es, en pequeño, el mismo problema que el
neurocontrolador tiene que resolver.

**(c) ✅ ESCALA DE PRESIÓN CALIBRADA — y el balance de fuerzas cierra.**
Calibración por dos puntos (`tools/daq.py --calibra-presion`):

| | canal A (`ai2`) | canal B (`ai3`) |
|---|---|---|
| **Punto 0** (UPH parada, manómetros a 0) | 1.9697 V | 1.9801 V |
| **Punto 1** (UPH en marcha, AO a −0.370 V, media de 180 s) | 2.8987 V | 4.7329 V |
| Manómetro en el punto 1 | **30 bar** | **50 bar** |
| **Escala resultante** | **32.293 bar/V** | **18.163 bar/V** |
| Fondo de escala a 10 V | 259.3 bar | 145.7 bar |

Que ambos canales den ~2.0 V a presión nula **confirma el span 4–20 mA sobre 500 Ω**
(2–10 V) que se había deducido en §2.4. Se usa el `V0` medido y no 2.000 V exactos: así la
recta absorbe también el offset del propio canal de entrada.

**La validación que de verdad cierra el asunto, y no es circular.** Con el pistón en
equilibrio y sin carga externa debe cumplirse `P_A·A_A = P_B·A_B`, es decir
**`P_B/P_A = A_A/A_B`**:

| | valor |
|---|---|
| Relación de áreas `A_A/A_B` (geometría) | **1.6410** |
| Relación de presiones `50/30` (manómetros) | **1.6667** |
| **Discrepancia** | **1.6 %** |

Las presiones se leyeron a ojo en dos manómetros analógicos y las áreas salen de la
geometría del cilindro: que coincidan al 1.6 % confirma **de una vez** las áreas asimétricas,
el reparto `ai2` = cámara grande / `ai3` = anular, y las propias lecturas. Es la relación de
**intensificación** de §2.3, medida directamente.

**Y el balance de fuerzas, que antes era absurdo, ahora cierra:**

| | escala supuesta | **escala calibrada** |
|---|---|---|
| `P_A·A_A` | 21.2 kN | 60.32 kN |
| `P_B·A_B` | 162.5 kN | 61.26 kN |
| Neto hidráulico | −141.3 kN | **−0.94 kN** |
| + peso del conjunto (~178 kg) | — | +1.75 kN |
| **Suma** | −141.3 kN | **+0.80 kN** |
| Celda de carga | −0.14 kN | −0.14 kN |

De **141 kN de disparate a 0.8 kN de residuo**, que es del orden de la fricción de sellos de
un cilindro Ø160. Las presiones ya son utilizables para el modelo y para el lazo de fuerza.

⚠ **Lo que no encaja: los fondos de escala.** Salen **259 y 146 bar**, y el laboratorio
reportó **0–100 y 0–400 bar**. La calibración empírica cierra la física, así que es la que
se usa, pero **conviene mirar la placa de los transductores**: o no son los del catálogo, o
los rangos anotados corresponden a otra cosa.

**(c2) La deriva residual en el null: −0.95 mm/min.** Sosteniendo el AO en −0.370 V durante
180 s, el vástago pasó de 96.76 a 93.92 mm. No es cero, y **es del mismo orden que el ensayo
de losa (1.5 mm/min)**. Dos lecturas, ambas útiles:
- El comando de velocidad nula real está algo más cerca de **−0.41 V** que de −0.370 V, o se
  ha movido con la temperatura del aceite durante la sesión.
- Más importante: **el punto de velocidad nula no se puede fijar en lazo abierto.** Un error
  de 40 mV en el null produce un error de velocidad comparable a la propia consigna del
  ensayo. Hace falta acción integral, y es un argumento más para la ley mixta PID+red.

**(d) Lección de procedimiento, aprendida moviendo el equipo.** El primer arranque se hizo
**sin forzar antes el AO a 0 V**, y el vástago se desplazó 15 mm solo: el NI 9263 **conserva
la última tensión escrita**, y la había dejado un panel de prueba de NI MAX. Presurizar es
dar energía a lo que ya esté pedido. `set_hpu` ahora **pone el AO a cero antes de energizar
la bomba**, siempre. Es la clase de detalle que no aparece en ninguna memoria.

### 5.4 La asimetría del cilindro: el resultado va al revés de lo que parece
Consecuencia directa del modelo de dos cámaras (`tools/planta_sim.py --check`), y una de las
cosas que la memoria no podía dar porque suponía el cilindro simétrico.

**La cuenta ingenua es falsa.** «La cámara anular tiene menos área, luego retraer será
1.64× más rápido» — no. El caudal no lo fija el área sino **el orificio**: al retraer, la
cámara **grande** tiene que evacuar `A_A·v` por el orificio de retorno, y *ese* es el cuello
de botella. Resolviendo el régimen (continuidad + balance de fuerzas + ecuación de orificio):

| Sentido | K velocidad | P_A régimen | P_B régimen |
|---|---|---|---|
| Extendiendo (P→A) | **0.902 mm/s por V** | 19.45 bar | 31.91 bar |
| Retrayendo (P→B) | **0.697 mm/s por V** | 50.06 bar | 82.15 bar |
| | **relación 0.772** | | |

**Retraer es más LENTO, no más rápido.** Y los datos de 2017 dan `0.0114/0.0147 = 0.78`
(§5.1): **coincide con 0.772 dentro del 1 %**. Es la primera predicción no trivial que el
modelo acierta contra datos medidos, y da confianza en la estructura de dos cámaras.

Nótese además que **retraer trabaja a presiones mucho más altas** (50/82 bar frente a
19/32 bar): la cámara anular se acerca a los 100 bar de la limitadora. Eso importa para la
seguridad de la captura y explica el rango de 400 bar de su transductor.

**Un error de modelado que costó encontrar, anotado para no repetirlo.** La primera versión
metía el `K_ce = 2e−12 m⁵/(N·s)` de la memoria como fuga física entre cámaras, y la relación
salía **0.363** en vez de 0.772. `K_ce` es el coeficiente **linealizado** caudal-presión:
agrupa la fuga real *más* la pendiente `dQ/dP` de la propia servoválvula. En un modelo no
lineal esa pendiente **ya la aporta la ecuación de orificio**, así que meter `K_ce` la cuenta
dos veces. Con `K_fuga = 0` el simulador reproduce exactamente la solución analítica.

### 5.6 Captura de identificación — ✅ hecha el 2026-08-13

Dos secuencias **independientes** (semillas 1 y 7), en la **misma sesión**, sin probeta,
emitidas con temporización por hardware:

| | `train` | `val` |
|---|---|---|
| Muestras / duración | 609 543 / 609.5 s | 612 094 / 612.1 s |
| **Ts (mín – máx)** | **1.0000 – 1.0000 ms** | **1.0000 – 1.0000 ms** |
| Recorrido | 71.6 – 99.4 mm | 42.9 – 77.0 mm |
| Comando | −9.95 … +8.43 V | −10.00 … +5.72 V |

**El `Ts` sale exacto hasta la última cifra**, mínimo y máximo idénticos. Es lo que da el
reloj del chasis y lo que ningún lazo de software puede garantizar (§2.5.1). Se emitieron
609 mil muestras sin perder ninguna y sin que saltara ningún límite.

**(a) La captura reproduce el barrido — validación cruzada con otra excitación.**
Ajustando la curva estática sobre los tramos de régimen de la propia captura:

| | captura APRBS | barrido de escalones | dif. |
|---|---|---|---|
| K⁺ | 0.4340 | 0.4459 | −2.7 % |
| K⁻ | 0.3766 | 0.3779 | −0.3 % |

R² = 0.999 en ambas ramas. Dos excitaciones de diseño completamente distinto dan la misma
planta.

**(b) Las dos capturas coinciden entre sí.**

| | `train` | `val` | dif. |
|---|---|---|---|
| K⁺ | 0.4340 | 0.4338 | −0.1 % |
| K⁻ | 0.3766 | 0.3711 | −1.5 % |

**(c) ⚠⚠ EL NULL SE MOVIÓ 71 mV ENTRE LAS DOS CAPTURAS — en diez minutos.**

| | |
|---|---|
| Comando de velocidad nula, `train` | **−0.356 V** |
| Comando de velocidad nula, `val` | **−0.285 V** |
| Desplazamiento | **+71 mV** ⇒ **1.58 mm/min** |

Son **~20 °C de aceite** según la especificación de Moog (0.20 V por 55 °C, §2.2) — un
calentamiento normal en una sesión con la bomba en marcha. Y en velocidad significa:

| Ensayo | consigna | la deriva del null es |
|---|---|---|
| Losa | 1.5 mm/min | **1.1× la consigna entera** |
| Viga | 0.1 mm/min | **16× la consigna** |

**En diez minutos de operación el punto de velocidad cero se movió más que la consigna
completa del ensayo de losa.** Tres consecuencias, y hay que tenerlas presentes al entrenar:

1. **`train` y `val` tienen nulls distintos.** Un NARX que aprenda la relación *absoluta*
   `u → velocidad` verá las dos series como incoherentes y promediará: el error de
   validación no bajará de ahí por mucho que crezca la red. No es sobreajuste, es que **las
   dos series describen plantas ligeramente distintas**.
2. **El modelo no debe apoyarse en un null absoluto.** Predecir incrementos (§3.3) ayuda
   pero no basta: el término independiente sigue estando. Opciones: meter la **temperatura
   del aceite** como entrada del regresor, o aceptar que el sesgo lo corrige el integrador
   del lazo.
3. **Refuerza la ley mixta PID+red** como hipótesis: la red aporta la anticipación y el
   integrador absorbe una deriva que **ningún modelo estático puede seguir**.

Y confirma con números la regla de §6.4: **comparar leyes de control en sesiones distintas
no tiene sentido en esta planta.**

**(d) Tercera validación de las presiones**, ahora sobre 610 s de movimiento:

| | |
|---|---|
| `P_A·A_A − P_B·A_B` (media) | −1.408 kN |
| Peso del conjunto móvil (~178 kg) | +1.75 kN |
| **Suma** | **+0.34 kN** |
| Celda de carga | −0.089 kN |

Los dos términos grandes casi se cancelan, como deben: a velocidades modestas y sin
aceleraciones grandes la fuerza neta ronda cero. El residuo es fricción de sellos. La celda
marca ~0 porque **sin probeta no hay reacción externa que medir**.

Ficheros: `results/captura_{train,val}.csv` · figura `results/captura_train.png`.
⚠ Los CSV pesan ~40 MB cada uno y **no se versionan** (ver `.gitignore`).

---

### 5.7 Fase 1 — modelo NARX, línea base (2026-08-13)

`tools/nn_modelo.py`. Red de una capa oculta escrita a mano con numpy, entrenada por Adam
con selección por reinicios sobre **simulación libre en datos no vistos**. Predice el
**incremento**, no la posición (§3.3).

**(a) El Ts del modelo no es el del control, y hay una razón medida.** El ruido de posición
(σ = 0.105 mm, §5.2) baja como √N al diezmar promediando, pero el incremento por muestra
crece con Ts. La relación señal/ruido del incremento a 1 mm/s:

| Ts | σ_y | \|dy\| a 1 mm/s | SNR |
|---|---|---|---|
| **20 ms** (el del control) | 0.0235 mm | 0.020 mm | **0.60** ← bajo el ruido |
| 50 ms | 0.0148 mm | 0.050 mm | 2.38 |
| **100 ms** (elegido) | 0.0105 mm | 0.100 mm | **6.73** |
| 200 ms | 0.0074 mm | 0.200 mm | 19.05 |

A 20 ms el incremento está **por debajo del ruido** justo en la banda de los ensayos. No se
pierde nada identificando más lento: a 50 Hz la servoválvula (120 Hz) y la resonancia
(>315 Hz) ya están sobre Nyquist.

**(b) ⚠ RESULTADO PRINCIPAL: el mejor modelo NO tiene realimentación.** Barrido de cuánta
historia de `dy` conviene realimentar:

| ny | un paso | libre train | **libre val** |
|---|---|---|---|
| **0** | 51.5 % | 51.5 % | **49.6 %** ← el mejor |
| 1 | 56.1 % | −133.5 % | **−440.9 %** ← diverge |
| 2 | 62.2 % | 37.8 % | 41.8 % |
| 3 | 62.1 % | 33.9 % | 31.9 % |
| 5 | 63.0 % | 44.1 % | 41.8 % |

**Cada `dy` realimentado mejora el ajuste a un paso y empeora la simulación libre.** La
causa es medible: `dy` lleva el ruido del sensor y `u` no lleva ninguno. Entrenando a un
paso, la red descubre que puede usar el ruido de `dy(k)` para predecir parte del ruido de
`dy(k+1)` — eso sube el «un paso» — pero en simulación libre esa entrada ya no es el ruido
medido sino **el error de la propia red**, que se realimenta y crece.

Y encaja con la física: a 100 ms no queda estado interno que recordar. **No hay nada que
realimentar.** El modelo adoptado es un **FIR no lineal**:

```
dy(k+1) = f( u(k), u(k−1) )        red 2-15-1
```

que además **no puede acumular error por construcción**: «un paso» y «simulación libre» son
la misma cuenta.

**(c) Resultados de la línea base:**

| | un paso | **sim. libre** | R² | sesgo | techo por ruido | alcanzado |
|---|---|---|---|---|---|---|
| train | 51.7 % | **51.7 %** | 0.767 | −0.40 mm/min | 65.1 % | **79 %** |
| val | 50.1 % | **50.1 %** | 0.751 | **+1.18 mm/min** | 57.7 % | **87 %** |

Error máximo de posición reintegrando en ventanas de 60 s: mediana **0.58 mm** (train) y
**1.13 mm** (val).

> **El 50 % hay que leerlo contra el techo, no contra el 100 %.** El techo es la fracción
> del incremento que **es ruido de medida**: ningún modelo puede superarlo. Se alcanza el
> **87 % de lo alcanzable** en validación. Y da una consecuencia accionable: **bajar el
> ruido de posición sube el techo directamente** — el tono de modo común a 133.8 Hz (§5.2b)
> deja de ser una curiosidad y pasa a ser la palanca de mayor retorno del proyecto.

**(d) ✅ El sesgo de validación ES la deriva del null, y coincide al 3 %.**

| | |
|---|---|
| Predicho desde el desplazamiento de 71 mV (§5.6c) | **1.581 mm/min** |
| Sesgo medido del modelo en validación | **1.632 mm/min** *(1.18 en la corrida final)* |

En `train` el sesgo es ≈ 0, como debe ser al evaluar sobre la misma planta con la que se
entrenó. **Confirma que no es sobreajuste ni falta de capacidad**: es que `train` y `val`
son plantas ligeramente distintas, y el modelo hereda la diferencia como sesgo constante.
Era exactamente lo que la opción (1) tenía que dejar a la vista.

**(e) Más capacidad empeora las cosas.** Con 30 neuronas y 400 épocas el «un paso» sube a
66.1 % y la **simulación libre cae a 17–22 %**. Es la misma trampa de (b) amplificada:
optimizar el criterio equivocado. **La métrica que manda es la simulación libre**, y hay
que seleccionar por ella.

### 5.8 Fase 1 — red dinámica entrenada por DBP, y comparación

Segundo enfoque del curso (Clase 07, punto B): modelo **recurrente** entrenado propagando
las sensibilidades **en el tiempo**, sin que la red vea nunca una medida.

```
h(k)   = tanh( [x(k), u(k)]·V + bv )
x(k+1) = h(k)·W + b
dy(k)  = x(k)[0]                      solo la 1ª componente se observa
```

**La motivación era buena y vale la pena dejarla escrita.** En el NARX serie-paralelo la red
recibe `dy(k)` **medido** como entrada, con su ruido: eso es un problema de *errores en las
variables* y sesga la estimación. En la configuración paralela del DBP la red recibe **su
propia salida**, determinista; el ruido queda solo en el objetivo, donde no sesga. Además el
DBP **entrena directamente sobre el error de simulación libre**, que es la métrica que
importa, en vez de entrenar a un paso y *seleccionar* por simulación libre (§5.7e).

**Gradiente verificado contra diferencias finitas: error relativo 1e-10 – 1e-11** en los
cuatro bloques de pesos. La comprobación quedó dentro del programa y se ejecuta en cada
entrenamiento.

**Resultado — los dos enfoques topan en el mismo sitio:**

| Modelo | estructura | **sim. libre val** | R² | sesgo val | % del techo |
|---|---|---|---|---|---|
| **Estático (FIR)** | 2-15-1, `f(u(k),u(k−1))` | **50.13 %** | 0.751 | +1.18 mm/min | **87 %** |
| **Dinámico (DBP)** | 3-10-3, estado ns=3 | **48.35 %** | 0.733 | +1.28 mm/min | **84 %** |
| *(techo por ruido)* | — | *57.7 %* | — | — | — |

**Y ése es el resultado, no cuál gana por dos puntos.** Dos arquitecturas con filosofías
opuestas —una sin memoria y entrenada a un paso, otra con estado interno y entrenada sobre
la trayectoria— llegan al mismo sitio, a unos 8 puntos del techo que impone el sensor.
**El límite no es la arquitectura: es el ruido de la medida de posición.** Cualquier
esfuerzo adicional en el modelo tiene rendimientos decrecientes; bajar σ sí los tiene.

Tres observaciones que conviene retener:

1. **El DBP no arregla la deriva del null**, y no podía: su sesgo en validación (+1.28) es
   el mismo que el del estático (+1.18). No es un defecto de estructura sino que `train` y
   `val` están a distinta temperatura, y **ningún estado recurrente puede inferir un
   desplazamiento del null que no tiene observable asociada**.
2. **En `train` el DBP tiene menos sesgo** (−0.05 vs −0.40 mm/min): era de esperar, porque
   optimiza exactamente el error de simulación libre. Su ventaja teórica es real; lo que
   pasa es que aquí no hay margen donde ejercerla.
3. **Contraste con `pi5_qnx_project`, donde el orden se invierte.** Allá el NARX ganó al DBP
   (82.6 % vs 76.8 %) porque recibía RPM medidas de buena calidad. Aquí esa ventaja se
   anula: la salida medida es ruidosa en relación con el incremento, y por eso el enfoque
   sin realimentación queda a la par. **La misma comparación da resultados opuestos en dos
   plantas, y la razón es la relación señal/ruido de la salida** — es más informativo que
   cualquiera de los cuatro números por separado.

⚠ **Corrección sobre una lectura preliminar.** Con 12 épocas los reinicios del DBP daban
46.2 % y 32.9 %, y se anotó que el entrenamiento recurrente tenía mucha más dispersión de
semilla. Con 40 épocas los tres reinicios dan 48.20 / 48.13 / 48.35: la dispersión era
**falta de convergencia**, no una propiedad del método.

---

### 5.9 Techo de caudal de la bomba y duración del transitorio (2026-08-13)

`tools/daq.py --satura`. Se recorre la carrera entera con el comando a fondo de escala y se
mira si la velocidad topa. Contesta dos cosas a la vez.

**(a) ✅ El caudal satura extendiendo: Q ≈ 3.18 L/min.**

| comando | v meseta | caudal implícito |
|---|---|---|
| 8 V | 2.536 mm/s | 3.06 L/min |
| **10 V** | **2.634 mm/s** | **3.18 L/min** |

El comando sube un **25 %** y la velocidad sólo un **3.9 %**: la válvula se abre más y no
entra más aceite. Sostenido 32 s, o sea **1.7 litros** de aceite desplazados — un acumulador
no puede aportar eso, así que es la bomba.

**(b) Retrayendo NO satura, y no es contradicción.** A 10 V da 2.711 mm/s = 1.99 L/min,
por debajo del techo. La razón es geométrica: retrayendo se llena la cámara **anular**, así
que con 3.18 L/min la velocidad sería 4.32 mm/s. Ese sentido sigue limitado por el orificio.

Por eso la relación entre sentidos no da una lectura limpia: **los dos están en regímenes
distintos**. Pasa de **0.946** (8 V) a **1.029** (10 V), moviéndose hacia `A_A/A_B = 1.641`
conforme entra en saturación — que es lo que predice el modelo. El discriminador funciona;
lo que estaba mal era esperar un único número con la planta a medio saturar.

⚠ **La placa de la UPH dice 1.7 L/min y lo medido es 1.9× esa cifra** (a 1710 rpm serían
1.86 cm³/rev). No se reabre la interpretación del dato de placa: para el modelo manda lo
medido. Poner 1.7 en `planta_sim.py` metería una saturación a 1.41 mm/s que la planta real
no tiene, y el simulador mentiría en toda la banda de comandos altos.

**Verificación tras actualizar el simulador:**

| u | v simulado | v medido |
|---|---|---|
| 8 V | 2.636 mm/s | 2.536 mm/s |
| 10 V | 2.636 mm/s | **2.634 mm/s** |

A fondo de escala coinciden al 0.1 %. La diferencia a 8 V es real y tiene explicación: la
saturación del equipo es **gradual** (2.536 → 2.634 entre 8 y 10 V) mientras que la del
modelo es un recorte duro. Suficiente para el uso que se le da.

**(c) ✅ El transitorio tras arrancar la bomba dura 2–3 MINUTOS.** Sosteniendo −0.370 V
durante 180 s:

```
deriva: +0.3202 -> -0.0227 mm/s
P_A   :  30.8   ->  30.8 bar  (asentada)
```

Es la medida que faltaba, y explica una racha de tropiezos del mismo día: la deriva de null
que se reportó y hubo que retirar, tres tanteos de sentido que daban 8× menos de lo previsto,
y dos abortos de la propia prueba. **Con 25 s no basta.** Va al protocolo: ninguna medida
corta vale antes de ~3 minutos de bomba en marcha.
