# ⚙️ Configurazione: Velocità Molto Bassa

## 🎯 Parametri Aggiornati

```yaml
target_velocity: 1.0 m/s    (era 3.0 m/s)
torque_per_wheel: 8.0 Nm    (era 12.0 Nm)
min_velocity: 0.5 m/s       (era 2.5 m/s)
max_velocity: 1.5 m/s       (era 3.5 m/s)
```

## 📊 Comportamento Atteso

### Velocità Media
- **Target**: ~1.0 m/s
- **Range operativo**: 0.5 - 1.5 m/s
- **Tempo giro stimato**: ~85 secondi (circonferenza 84.8m / 1.0 m/s)

### Controllo Torque

```
Velocità:
   1.5 m/s ════╗ Soglia superiore → 🔴 SPEGNE torque
               ║
   1.0 m/s     ║ ZONA NEUTRA
               ║ (mantiene stato precedente)
   0.5 m/s ════╝ Soglia inferiore → 🟢 ACCENDE torque
```

### Timeline Esempio

```
t=0.0s  v=0.0 m/s  → 🟡 WARMUP (rampa torque 0→100%)
t=1.0s  v=0.3 m/s  → 🟡 WARMUP
t=2.0s  v=0.6 m/s  → 🟡 WARMUP
t=3.0s  v=0.8 m/s  → 🟢 ON (v > min, accelera ancora)
t=5.0s  v=1.2 m/s  → 🟢 ON (in range, mantiene)
t=7.0s  v=1.6 m/s  → 🔴 OFF (v > max_velocity!)
t=10.0s v=1.3 m/s  → 🔴 OFF (in range, mantiene)
t=13.0s v=0.9 m/s  → 🔴 OFF (in range, mantiene)
t=16.0s v=0.4 m/s  → 🟢 ON (v < min_velocity!)
... ciclo continua
```

## 🎮 Vantaggi Velocità Bassa

✅ **Massima precisione**: Più tempo per correggere traiettoria
✅ **Controllo superiore**: Errori più facili da gestire
✅ **Meno forze**: Meno stress meccanico, meno rischio sbandamento
✅ **Debug facile**: Più facile osservare cosa succede
✅ **Sicurezza**: Ideale per test e tuning

## 🔧 Tuning Fine

### Se la macchina si ferma:

```yaml
# Opzione 1: Aumenta torque
torque_per_wheel: 10.0  # invece di 8.0

# Opzione 2: Alza soglia minima
min_velocity: 0.7  # invece di 0.5

# Opzione 3: Entrambi
torque_per_wheel: 10.0
min_velocity: 0.7
```

### Se va troppo veloce:

```yaml
# Opzione 1: Riduci torque
torque_per_wheel: 6.0

# Opzione 2: Abbassa soglia massima
max_velocity: 1.2  # invece di 1.5

# Opzione 3: Entrambi
torque_per_wheel: 6.0
max_velocity: 1.2
```

### Per velocità ancora più bassa:

```yaml
torque_per_wheel: 5.0
min_velocity: 0.3
max_velocity: 0.8
# Velocità media: ~0.5 m/s (molto lenta!)
```

## 📐 Calcoli Teorici

### A 1.0 m/s:

- **Velocità angolare**: ω = v/R = 1.0/13.5 ≈ 0.074 rad/s ≈ 4.24°/s
- **Tempo per giro completo**: T = 2πR/v = 84.8/1.0 ≈ 85 secondi
- **Accelerazione centripeta**: a = v²/R = 1.0²/13.5 ≈ 0.074 m/s²

### Confronto con Velocità Precedente:

| Parametro | Velocità Alta (3.0 m/s) | Velocità Bassa (1.0 m/s) |
|-----------|-------------------------|--------------------------|
| Tempo giro | ~28 secondi | ~85 secondi |
| Accel. centripeta | 0.67 m/s² | 0.074 m/s² |
| Forza laterale | 119 N | 13 N |
| Precisione | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

## 🚀 Come Applicare

```bash
# I file sono già modificati! Basta ricompilare e lanciare:

cd /home/feld/Documents/e-team/ros2_ws
colcon build --packages-select circular_controller
source install/setup.zsh
ros2 launch circular_controller circular_controller_feedback.launch.py
```

## 📱 Output Atteso

```
[INFO] === Circular Controller with Feedback ===
[INFO] Target velocity: 1.00 m/s
[INFO] Velocity control: 0.50 - 1.50 m/s
[INFO] Torque per wheel: 8.00 Nm
[INFO] =======================================

[INFO] 🟢 TORQUE ON: v=0.82 m/s < 1.50 m/s (max)
[INFO] t:5.0s | v:1.15 m/s | R:13.20m (err:-0.30m) | Torque:ON 🟢 100.0%
[INFO] 🔴 TORQUE OFF: v=1.58 m/s > 1.50 m/s (max)
[INFO] t:8.0s | v:1.32 m/s | R:13.15m (err:-0.35m) | Torque:OFF 🔴 0.0%
[INFO] t:12.0s | v:0.95 m/s | R:13.42m (err:-0.08m) | Torque:OFF 🔴 0.0%
[INFO] 🟢 TORQUE ON: v=0.48 m/s < 0.50 m/s (min)
```

## ⚠️ Possibili Problemi

### Problema: La macchina si ferma completamente

**Causa**: Torque troppo basso per vincere attrito statico

**Soluzione**:
```yaml
torque_per_wheel: 10.0  # aumenta
min_velocity: 0.7       # non lasciare rallentare troppo
```

### Problema: Oscillazioni continue ON/OFF

**Causa**: Range troppo stretto o torque troppo alto

**Soluzione**:
```yaml
min_velocity: 0.4
max_velocity: 1.6
# Gap aumentato da 1.0 a 1.2 m/s
```

### Problema: Non raggiunge mai 1.5 m/s

**Causa**: Torque insufficiente

**Soluzione**:
```yaml
torque_per_wheel: 12.0
warmup_time: 4.0  # più tempo per accelerare
```

## 💡 Raccomandazioni

1. ✅ **Inizia con questi valori** e osserva 1-2 giri completi
2. ✅ **Monitora i log** per vedere frequenza switch ON/OFF
3. ✅ Se oscilla troppo → aumenta gap (es. 0.4-1.6 m/s)
4. ✅ Se troppo lenta → aumenta torque a 10 Nm
5. ✅ **Feedback gain**: considera ridurre a 0.3-0.4 per velocità basse

## 🎯 Configurazione Alternativa (Ultra-Lenta)

Se 1.0 m/s è ancora troppo veloce:

```yaml
target_velocity: 0.6
torque_per_wheel: 5.0
min_velocity: 0.3
max_velocity: 0.9
warmup_time: 4.0
feedback_gain: 0.3
```

Questo darà velocità media di ~0.6 m/s (tempo giro ~140s).

---

**TL;DR**: Configurazione aggiornata per velocità 0.5-1.5 m/s (molto più lenta). Movimento più controllato e preciso! 🐌🎯
