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


## Data collection 

src/aquabot_bringup/launch/collect_dataset.sh lancia ( 4 + 3 ) * REPS trial e salva i dati in /logs per il pretraining. 4 a freq fissa e amp_sweep , 3 viceversa. 
NB. prima va lanciato system_launch.py

root in aquabot

# Rete
source .venv/bin/activate
python3 src/net/SensorEstimator/dataset.py ./src/net/dataset ./src/net/scaler/scalers.pkl
python3 src/net/SensorEstimator/train.py


python src/net/SensorEstimator/channel_loss.py
 
 
## Tuning dei parametri 
./src/net/SensorEstimator/run_tuning.sh <fase 1 - 2 - 3> [n_worker]
python3 src/net/SensorEstimator/tuning_status.py --watch


## Inversa 
python3 src/net/InverseEstimator/dataset_inverse.py ./src/net/dataset ./src/net/scaler/scalers_inverse.pkl
python3 src/net/InverseEstimator/train_inverse.py 

## Tuning parametri  
./src/net/InverseEstimator/run_tuning_inverse.sh <fase 1 - 2 - 3> [n_worker]
python3 src/net/InverseEstimator/tuning_status_inverse.py --watch

# Check loss 
python3 src/net/SensorEstimator/checkpoints/real_results.py --scaler ./src/net/scaler/scalers.pkl --mse-json ./src/net/SensorEstimator/checkpoints/channel_loss.2json

# Real values 
python3  ./src/net/SensorEstimatos/checkpoints/real_results.py --scaler ./src/net/scaler/scalers.pkl --mse-json ./src/net/SensorEstimatos/checkpoints/channel_loss.json

# Cosa fare ora
- RIPARTIRE DA : 
- rileggere slide  
- riguardare codice 


- dataset nuovo grande generato con formula unica e variazioni rendomiche dei tre parametri di generazione della sinusoide 
- evidenziare in una slide la differenza nel valore della corrente per un trial con e senza coda
- Tre versioni di allenamento: 1 con FM e IM separati senza autoregressione, 1 con FM e IM separati con autoregressione ed una con i modelli collegati (output di FM che entra nel contesto di IM e viceversa). Tutti e tre con test open loop e closed loop + metriche 
- fare il tuning tutto in un unico grosso passo e non più tre fasi 
- aggiungere gli interevalli in cui ho fatto variare i parametri nella generazione del dataset  
- aggiungere l'autocorrelazione nel test closed loop 
- esplicitare come sono calcolati i valori delle metriche delle tabelle  
- normalizzare per il numero di campioni le curve di training e validation set nel plot della loss , fare anche log scaling sull'asse y 
- nei grafici open e closed loop plottare anche la differenza tra segnale reale  e segnare predetto 
- quando P > 1 fare un grafico che evidenzi  che per ogni volta che un certo valore viene predetto , il valore predetto si avvicina sempre di più alla media 

- attenzione alla calibrazione sui primi 50 campioni, si fa così? 


# RIASSUNTO 
Progetto: modello congiunto IM+FM per robot-pesce 

Architettura. Due reti GRU+MLP (model_joint.py) che condividono lo stesso ingresso [C_1:H, S_1:H, ctx] — storia di H=20 comandi + H sensori + contesto. Testa singola a P passi (P=1 ora). IM (inversa) predice il comando, FM (diretta) predice i sensori [sensor_diff, current]. Contesto = [amp, freq, center, dt], CTX_DIM=4. Scaler unico condiviso (scalers_joint.pkl), split per-trial leak-free.

File pronti: model_joint.py (con dropout), dataset_joint.py, train_joint.py (loss IM/FM separate, dropout+weight_decay), tune_joint.py (tuning 3 fasi solo FM), plot_prediction_joint.py (open-loop, deduce dimensioni dal checkpoint), closed_loop_test_joint.py, master_node.py (formula unica, solo std/sweep/turning, log con center_rad + real_position_rad), collect_dataset.sh (55 trial), trial_launch.py, plot_trial.py.

Stato: Fase A completata. Training congiunto funziona, IM va a zero (comando facile), FM overfittava ma il tuning con dropout+weight_decay ha chiuso il gap train/val. Modello robusto. Open-loop predictions buone.

Prossimo passo: Fase B — implementare la cycle_loss in train_joint.py (già predisposta, ora è NotImplementedError) e accendere lambda_cyc con warm-up. Il closed_loop_test_joint.py è lo strumento per misurare se il ciclo migliora la tenuta dell'anello.

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