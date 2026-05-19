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

python3 src/net/dataset.py ./logs --checkpoint_dir checkpoints/
python3 src/net/train.py --log_dir ./logs --checkpoint_dir ./checkpoints

## Inversa 
python3 src/net/dataset_inverse.py ./logs --checkpoint_dir checkpoints/
python3 src/net/train_inverse.py --log_dir ./logs --checkpoint_dir ./checkpoints

# Cosa fare ora
- attenzione alla calibrazione sui primi 50 campioni, si fa così? 
- la rete funziona? 
- fare le slide con la rete nuova -  sistemare grafici e formule loss ecc.. 
- si chiama ancora v_flow ma nel codice è corrente motore, cambiare quando siamo sicuri
- ATTENZIONE l'ultima volta i sensori non hanno letto niente! CONtrolla 
