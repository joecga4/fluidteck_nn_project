# Protocolo de Fase 0 — primera sesión de máquina

> Objetivo: dejar caracterizada la cadena (sentido, ganancia real, deriva de null,
> latencia) y capturar los dos conjuntos de datos para identificación.
> Todo lo que aquí se mide se anota en `CLAUDE.md §5`.
>
> **Requisito no negociable:** alguien presente con acceso a la seta de emergencia.

---

## 0. Antes de tocar nada

- [ ] **Cerrar LabVIEW** y **cerrar los paneles de prueba de NI MAX**.
      LabVIEW y MAX reservan los módulos y bloquean a Python. Si aparece
      `resource already reserved`, es esto. Comprobar con `--diag`.
- [ ] **Sin probeta.** La identificación se hace con el vástago libre.
- [ ] Verificar la **seta de emergencia** y que la limitadora está a 100 bar.
- [ ] Anotar la **temperatura del aceite** (indicador de la UPH). La viscosidad
      cambia la ganancia; sin este dato las sesiones no son comparables — es la
      lección que costó tres sesiones en el proyecto del motor.

```
python tools/daq.py --diag        # SOLO LECTURA: módulos + 4 canales AI
python tools/daq.py --di          # SOLO LECTURA: enclavamientos
```

**Qué esperar en `--di` con la UPH apagada** (medido el 2026-08-12):
`line0 = 1`, `line1 = 1`, resto `0`. Probablemente contactos normalmente cerrados
(«seta no pulsada», «filtro no saturado»), pero **sin confirmar**.

---

## 1. Identificar la línea de arranque de la UPH  ← *pendiente*

El NI 9472 lleva **el arranque del motor y la parada de emergencia desde la
pantalla** (memoria §2.5). No está documentado qué línea es cada cosa, y no está
ni en los VIs (comprimidos) ni en NI MAX (sin tareas guardadas). **No se adivina.**

Dónde buscarlo, por orden de fiabilidad:
1. **Plano eléctrico del tablero** (bornes del NI 9472 a los relés).
2. **Diagrama de bloques de `HPU.vi`** abierto en LabVIEW: el nombre del canal
   DAQmx aparece como constante o control.
3. Etiquetado físico en el módulo.

Una vez identificada, anotarla en `CLAUDE.md §2.5.1` y usarla siempre explícita:

```
python tools/daq.py --hpu on --linea-hpu N --armar
```

`set_hpu` lee las DI **antes y después** y luego los sensores: si encender la UPH
no cambia ninguna DI, o la línea no es la correcta o el arranque no prosperó.

---

## 2. Confirmar la escala de presión  (2 minutos, cierra una incógnita)

Con la UPH **en marcha** y el comando a 0 V:

```
python tools/daq.py --sensores --secs 15
```

- [ ] Comparar `presion_A` contra el **manómetro** de la cámara A.
- [ ] Si coinciden → queda confirmado el **span 2–10 V** deducido en `CLAUDE.md §2.4`.
- [ ] Si no coinciden, anotar ambos valores: la escala está mal y todo el lazo de
      fuerza depende de ella.

---

## 3. Sentido, ganancia real y deriva de null  ← *la medida que más rinde*

```
python tools/daq.py --caracteriza --armar
```

Hace, por este orden: tanteo de **sentido** (+0.3 V, 1.5 s) → **recolocación** a
75 mm → barrido `0, ±0.05, ±0.1, ±0.25, ±0.5, ±1, ±2 V`, 2.5 s cada uno, con
vigilancia de límites cada 50 ms y recolocación si deriva más de 15 mm.

Anotar en `CLAUDE.md §5`:
- [ ] **Sentido:** ¿un comando positivo aumenta o disminuye la posición medida?
- [ ] **K positiva** y **K negativa** (mm/s por V).
- [ ] **Asimetría |K−/K+|.** El modelo de dos cámaras predice **0.772** y los datos
      de 2017 dieron **0.78**. Si sale cerca, el modelo está bien planteado.
- [ ] **Deriva de null** (el paso con u = 0). En 2017 era ~+0.03 mm/s.
- [ ] **Presiones de régimen** en cada sentido. El modelo predice ~19/32 bar
      extendiendo y ~50/82 bar retrayendo (§5.4).

**Esto cierra la discrepancia de ~6×** entre el modelo físico y los datos de 2017
(`CLAUDE.md §8`), que es la incógnita más grande que queda.

---

## 4. Latencia del lazo cerrado

```
python tools/daq.py --latencia --armar     # escribe 0 V: no mueve la planta
```

- [ ] Anotar mediana, p95 y máximo del tiempo leer+escribir.
- [ ] De ahí sale el **Ts de control alcanzable** y el **retardo puro** que hay que
      meter en el modelo NARX.

---

## 5. Regenerar la excitación con la ganancia medida

Las secuencias actuales se diseñaron con la K del modelo (0.90 mm/s·V). Con la K
real medida en el paso 3:

```
python tools/gen_excitacion.py --seed 1 --etiqueta train --K <K_medida> --plot
python tools/gen_excitacion.py --seed 7 --etiqueta val   --K <K_medida> --plot
```

- [ ] Verificar que el informe dice `[ok] la secuencia se mantiene dentro de la ventana`.

---

## 6. Captura

```
python tools/daq.py --jog 75 --armar                                   # recolocar
python tools/daq.py --captura results/excitacion_train.csv --etiqueta train --armar
python tools/daq.py --jog 75 --armar
python tools/daq.py --captura results/excitacion_val.csv --etiqueta val --armar
```

Cada captura son ~10 minutos. Ejecutar **sin `--armar` primero**: hace todas las
comprobaciones previas y no escribe nada.

Al terminar, comprobar antes de dar la captura por buena:
- [ ] El recorrido de posición coincide con el previsto por `gen_excitacion`.
- [ ] No hay tramos planos sospechosos (sensor saturado o perdido).
- [ ] **Temperatura del aceite al inicio y al final.**
- [ ] `train` y `val` en la misma sesión y a temperatura parecida.

---

## 7. Parar

```
python tools/daq.py --cero --armar
python tools/daq.py --hpu off --linea-hpu N --armar
```

---

## Si algo va mal

| Síntoma | Causa probable |
|---|---|
| `resource already reserved` | LabVIEW o un panel de prueba de MAX tiene el módulo |
| «el vástago no se mueve» en el tanteo | UPH parada, amplificador sin alimentar, sin presión de suministro, o seta pulsada |
| Aborta por posición fuera de rango | El vástago no está en `[20, 130] mm`. Recolocar con `--jog` |
| La velocidad medida no se parece a la esperada | Es justo lo que se va a medir: anotarlo, no «corregirlo» |
| Ctrl-C durante una captura | El AO se lleva a 0 V y se guardan los datos hasta el corte |
