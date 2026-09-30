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
- riguardare codice 
- aggiungere l'autocorrelazione nel test closed loop 
- attenzione alla calibrazione sui primi 50 campioni, si fa così? 
- agggiungere test set?  

CHIARIRE 
- riguardare codice 


# Requirements 
uv pip install -r requirements.txt

