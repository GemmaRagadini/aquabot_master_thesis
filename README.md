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
python3 src/net/train_inverse.py --dataset_dir ./src/net/dataset --checkpoint_dir ./checkpoints

## Tuning parametri  
./src/net/InverseEstimator/run_tuning_inverse.sh <fase 1 - 2 - 3> [n_worker]
python3 src/net/InverseEstimator/tuning_status_inverse.py --watch

# Check loss 
python3 src/net/SensorEstimator/checkpoints/real_results.py --scaler ./src/net/scaler/scalers.pkl --mse-json ./src/net/SensorEstimator/checkpoints/channel_loss.json

# Real values 
python3  ./src/net/SensorEstimatos/checkpoints/real_results.py --scaler ./src/net/scaler/scalers.pkl --mse-json ./src/net/SensorEstimatos/checkpoints/channel_loss.json

# Cosa fare ora
- RIPARTIRE DA : 
- prova a vedere cosa succede con il training senza amp e freq 
- attenzione alla calibrazione sui primi 50 campioni, si fa così? 
- slide 
- entrare più nel dettaglio del training sulle slide 
- mettere TS/VS su slide in dataset 


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