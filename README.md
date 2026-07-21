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


# Rete
python3 src/net/dataset.py ./logs/ds --checkpoint_dir checkpoints/
python3 src/net/train.py --log_dir ./logs/ds --checkpoint_dir ./checkpoints

## Inversa 
python3 src/net/dataset_inverse.py ./logs/ds --checkpoint_dir checkpoints/
python3 src/net/train_inverse.py --log_dir ./logs/ds --checkpoint_dir ./checkpoints

# Tuning dei parametri 

## Fase 1 – griglia architetture 
python3 src/net/tune.py --phase 1 --storage sqlite:///tuning_results/optuna_fish.db

## Fase 2 – esplorazione sparsa parametri training 
python3 src/net/tune.py --phase 2 --storage sqlite:///tuning_results/optuna_fish.db

## Fase 3 – tuning finale di tutto 
python3 src/net/tune.py --phase 3 --storage sqlite:///tuning_results/optuna_fish.db

# Cosa fare ora
- attenzione alla calibrazione sui primi 50 campioni, si fa così? 
- sistemare le slide con le reti. (vedi foglio)
- MODEL SELECTION: 
      - tenere tutto fisso , variare solo gru_hidden e mlp_hidden scegliendo dei range 
      - aumentare molto il numero di parametri ma testando poche combinazioni 
      - optuna finale
- non sto usando amp  e freq nella rete . ok?
- specificare il dataset nelle slide  
- una volta che il modello allenato in fase di test fare un check per vedere se il valore nel futuro coincide con quello che effettivamente arriverà a t+1

# Requirements 
uv pip install -r requirements.txt