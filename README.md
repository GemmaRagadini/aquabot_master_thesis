# Aquabot – ROS 2 Workspace

ROS 2 workspace for the Aquabot Master Thesis.

Il master genera traiettorie base per l'oscillazione della coda. Per adesso le modalità sono: 
- std (frequenza e ampiezza costante) 
- freq_sweep ( ampiezza costante ma frequenza che aumenta e diminuisce)
- amp_sweep (viceversa) 
- 1to1 => il motore cambia posizione sulla base dei valori percepiti dei sensori


Se *Feedback_enabled* è true durante una delle traiettorie viene cambiato il centro di oscillazione (applicando una correzione al bias della sinusoide) quando viene percepita variazione dai sensori


## Packages
- aquabot_bringup
- arduino_reader
- dynamixel_controller
- master

## Build
- colcon build
- source install/setup.bash 

## Launch
ros2 launch aquabot_bringup system_launch.py

# write csv 
ros2 service call /trial std_srvs/srv/SetBool "{data: true}"

root in aquabot

# 4 TRAINING  
1.supervised — la Fase A, il riferimento.
2.supervised + --detach_cross — ablation: serve il cross-gradient o no?
3.rollout — closed-loop differenziabile.
4. combo — supervised + λ·rollout con warm-up.

# Cosa fare ora
- RIPARTIRE DA : 
- rileggere slide  
- riguardare codice 


- evidenziare in una slide la differenza nel valore della corrente per un trial con e senza coda
- fare il tuning tutto in un unico grosso passo e non più tre fasi 
- nelle slide aggiungere gli interevalli in cui ho fatto variare i parametri nella generazione del dataset  
- aggiungere l'autocorrelazione nel test closed loop 
- esplicitare come sono calcolati i valori delle metriche delle tabelle  
- quando P > 1 fare un grafico che evidenzi  che per ogni volta che un certo valore viene predetto , il valore predetto si avvicina sempre di più alla media 

- attenzione alla calibrazione sui primi 50 campioni, si fa così? 


1. Screening dei candidati — solo metriche, niente plot ancora.
Alleni le poche varianti che vuoi confrontare (combo, magari rollout, l'ablation detach_cross, eventualmente uno o due λ). Per ognuna guardi solo la tabella di metrics_per_trial_joint.py con --steps first avg last. Qui non ti servono ancora i plot: la tabella basta a dire chi vince. È veloce e ti evita di generare grafici per modelli che poi scarti.

2. Scelta del vincitore.
Dal master CSV scegli il modello migliore secondo il criterio che conta per te (di solito: skill vs persist positivo sul closed-loop + RMSE basso a last per il diretto a P passi). Uno solo.

3. Tuning — ma solo sul vincitore.
Qui sta l'inversione rispetto a come l'hai messo tu: il tuning viene dopo lo screening, non prima. Non ha senso ottimizzare λ o gli hidden su un'architettura che poi non scegli. Fai il tuning fine solo sulla configurazione che ha vinto lo screening.

4. Riallenamento finale.
Il run "buono" definitivo, con i parametri scelti, tag pulito, magari epoche più lunghe se non era a plateau.

5. SOLO ORA i plot, tutti insieme.
Curve di training, plot delle predizioni (vero vs predetto nel tempo), tabella metriche finale. Li generi in blocco, con lo stesso stile, alla fine, così sulle slide sono coerenti tra loro.


# Requirements 
uv pip install -r requirements.txt

# Iperparametri tuning finale 

# Per tornare a 3 canali output (con anche sensor_mean)
1. In model.py: cambia la costante in cima da N_OUTPUTS = 2 a N_OUTPUTS = 3. Basta quella riga, le due teste si adeguano da sole.

2. In dataset.py, dentro _build_windows: togli il commento dalle due righe sm_n. Cioè da così:

python
target_history = np.stack([
    sd_n[i - h:i],
    # sm_n[i - h:i],
    vf_n[i - h:i],
], axis=1)

target_future = np.array([sd_n[i + 1], vf_n[i + 1]], dtype=np.float32)

torni a così:

python
target_history = np.stack([
    sd_n[i - h:i],
    sm_n[i - h:i],
    vf_n[i - h:i],
], axis=1)

target_future = np.array([sd_n[i + 1], sm_n[i + 1], vf_n[i + 1]], dtype=np.float32)