# Circular Controller per pacsim

Questo package ROS2 implementa **due controller** per far seguire al veicolo simulato in pacsim una traiettoria circolare precisa.

## 🎮 Controller Disponibili

### 1. **Controller Open-Loop** (`circular_controller_node`)
- Controller semplice con comandi costanti
- Utilizza solo la cinematica del veicolo
- Include compensazione empirica per ridurre la tendenza a stringere la curva
- **Uso**: test rapidi, tuning iniziale

### 2. **Controller con Feedback** (`circular_controller_feedback_node`) ⭐
- Legge la posizione del veicolo tramite TF
- Corregge in tempo reale l'angolo di sterzo basandosi sull'errore radiale
- Mantiene il raggio costante compensando derive e disturbi
- **Uso**: massima precisione di traiettoria

## Descrizione

Il controller calcola l'angolo di sterzo necessario basandosi sulla cinematica del modello a bicicletta e pubblica comandi costanti di sterzo e torque per mantenere il veicolo su un percorso circolare di raggio specificato.

### Calcoli Cinematici

Per un raggio `R = 13.5 m` e passo del veicolo `L = 1.5 m`:

- **Angolo di sterzo ruote**: δ = arctan(L/R) = arctan(1.5/13.5) ≈ 0.1107 rad ≈ 6.34°
- **Angolo volante**: δ_wheel = δ / inner_steering_ratio ≈ 0.433 rad ≈ 24.8°
- **Velocità angolare attesa**: ω = v/R (es. 5 m/s → 0.37 rad/s)
- **Circonferenza**: C = 2πR ≈ 84.8 m
- **Tempo giro atteso**: T = C/v ≈ 17 s (a 5 m/s)

## Topics

### Pubblicati
- `/pacsim/steering_setpoint` (pacsim/msg/StampedScalar): Angolo del volante
- `/pacsim/torques_max` (pacsim/msg/Wheels): Torque massimo per ogni ruota

## Parametri Configurabili

### Controller Open-Loop (`config/circular_params.yaml`)

- `circle_radius` (double, default: 13.5): Raggio del cerchio [m]
- `target_velocity` (double, default: 5.0): Velocità target [m/s]
- `torque_per_wheel` (double, default: 15.0): Torque per ruota [Nm]
- `warmup_time` (double, default: 2.0): Tempo di ramp-up [s]
- `publish_rate` (double, default: 50.0): Frequenza pubblicazione [Hz]
- `steering_compensation_factor` (double, default: 0.92): **Fattore di compensazione empirico**
  - < 1.0: riduce angolo → raggio più largo (se la macchina stringe troppo)
  - > 1.0: aumenta angolo → raggio più stretto
  - Regola questo valore se noti che la traiettoria non è precisa

### Controller con Feedback (`config/circular_feedback_params.yaml`)

Tutti i parametri sopra, più:
- `feedback_gain` (double, default: 0.5): Guadagno proporzionale (0.3-1.0)
- `center_x`, `center_y` (double): Coordinate del centro del cerchio [m]

## Build & Run

### Compilazione

```bash
cd /home/feld/Documents/e-team/ros2_ws
colcon build --packages-select circular_controller
source install/setup.zsh
```

### Esecuzione

**Controller Open-Loop (semplice):**
```bash
# Con launch file
ros2 launch circular_controller circular_controller.launch.py

# Nodo diretto
ros2 run circular_controller circular_controller_node
```

**Controller con Feedback (raccomandato per precisione):**
```bash
# Con launch file
ros2 launch circular_controller circular_controller_feedback.launch.py

# Nodo diretto
ros2 run circular_controller circular_controller_feedback_node
```

**Con parametri personalizzati:**
```bash
ros2 run circular_controller circular_controller_node \
  --ros-args \
  -p circle_radius:=10.0 \
  -p target_velocity:=3.0 \
  -p torque_per_wheel:=12.0 \
  -p steering_compensation_factor:=0.95
```

## Uso con pacsim

1. Avviare pacsim in una finestra di terminale
2. In un altro terminale, avviare questo controller
3. Il veicolo inizierà a seguire il percorso circolare

### Tuning

#### Controller Open-Loop

Se il veicolo:
- **Stringe troppo la curva a metà percorso** (passa sopra i coni interni):
  - ✅ **RIDUCI** `steering_compensation_factor` (es. da 0.92 a 0.88-0.90)
  - ✅ Diminuisci `target_velocity`
  - ✅ Riduci `torque_per_wheel`
  
- **Allarga troppo** (esce fuori dai coni esterni):
  - ✅ **AUMENTA** `steering_compensation_factor` (es. da 0.92 a 0.95-0.98)
  - ✅ Aumenta leggermente il torque

- **Non raggiunge la velocità**: aumentare `torque_per_wheel` (15→20-25 Nm)
- **Sbanda o va fuori traiettoria**: diminuire `target_velocity` o `torque_per_wheel`
- **Accelera troppo bruscamente**: aumentare `warmup_time`

#### Controller con Feedback

Se il veicolo:
- **Oscilla troppo**: riduci `feedback_gain` (0.5 → 0.3)
- **Non corregge abbastanza**: aumenta `feedback_gain` (0.5 → 0.7-0.8)
- **Verifica che `center_x` e `center_y` siano corretti** guardando la posizione iniziale

## Note Tecniche

### Perché la Macchina Stringe a Metà Percorso?

Hai osservato correttamente che con il controller open-loop la macchina tende a:
1. Seguire bene il percorso all'inizio
2. Stringere troppo dopo ~1/4 di giro
3. Tornare verso la traiettoria corretta alla fine

**Cause principali:**

1. **Accumulo di forze laterali**: Le forze laterali sui pneumatici aumentano durante la curva sostenuta, creando un angolo di sideslip che si accumula nel tempo

2. **Velocità che aumenta**: Anche con torque costante, la velocità può aumentare leggermente → velocità angolare maggiore → raggio più stretto

3. **Effetti aerodinamici**: Downforce cresce con v² → più grip → raggi più stretti possibili

4. **Modello cinematico vs dinamico**: Il modello forza `vy = 0` ma calcola comunque le forze laterali, creando inconsistenze

**Soluzioni:**
- **Soluzione rapida**: regola `steering_compensation_factor` nel file YAML (già implementata)
- **Soluzione migliore**: usa il controller con feedback che corregge in tempo reale

### Dettagli Implementazione

- Controller open-loop con **compensazione empirica** dello sterzo
- Controller con feedback usa **controllo proporzionale** sull'errore radiale
- Durante warmup (primi 2 secondi default):
  - 0-0.5s: accelerazione in linea retta
  - 0.5s-warmup_time: aumento graduale dello sterzo
- Dopo il warmup: comandi costanti (open-loop) o corretti (feedback)
- Il modello del veicolo in `VehicleModelBicycle.cpp` forza `vy = 0` (modello cinematico puro)

## Author

E-Team Unipi
