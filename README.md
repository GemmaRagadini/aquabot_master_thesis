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


# Cosa fare ora
- RIPARTIRE DA : 
- riguardare codice 

# Suggerimenti 

Detection

    Lo spostamento minimo che fa scattare l'allarme è circa 0.7 std su sensor_diff (circa 110 unità) e 1.8 std sulla corrente (circa 100 mA). Sono valori abbastanza alti: per esempio nella slide con e senza pinna la differenza di corrente è di circa 30 mA, e solo sui picchi. Tienilo presente quando scegli le perturbazioni da provare.
    Le perturbazioni sintetiche che hai usato sono offset costanti, mentre quelle fisiche cambiano più che altro ampiezza, fase e smorzamento. Aggiungerei qualche test sintetico con una variazione di guadagno e con un piccolo ritardo, così ti avvicini a quello che vedrai sul robot.
    La soglia è calibrata su 24 trial da 30 s, cioè circa 12 minuti di dati. Sul robot ti servirà capire quanti falsi allarmi hai su tempi più lunghi.

Test sul robot (suggerimenti)

    Prima prova senza nemmeno accendere il robot: hai già le registrazioni con e senza pinna. Passale offline nel detector e guarda se e quando scatta l'allarme. È il primo test su una perturbazione vera.
    Registra alcune sessioni in condizioni normali, in giorni diversi, da 30-60 minuti, sia con i comandi random sia con le modalità del master (std, sweep). Da qui ricavi i falsi allarmi per ora. Se il robot cambia già da un giorno all'altro, meglio scoprirlo subito.
    Poi le perturbazioni fisiche, segnando sempre l'istante esatto in cui le applichi (per esempio un topic ROS con un marker, o un tasto, e se puoi un video sincronizzato). Qualche idea:

    persistenti: togliere o cambiare la pinna, aggiungere masse crescenti in punta;
    transitorie: trattenere la coda con la mano o mettere un ostacolo, con durate diverse intorno ai 5 s (es. 2, 4, 6, 10 s), così metti alla prova la distinzione tra transitorio e persistente;
    graduali: qualche prova lunga, visto che la motivazione della tesi è proprio il degrado nel tempo.
    Per ogni tipo farei una decina di ripetizioni, con istanti di inizio diversi.

    Registra tutto e fai l'analisi prima offline. Il detector è deterministico, quindi puoi regolare k, W e T senza rifare le prove. Tieni separate le sessioni che usi per calibrare da quelle che usi per valutare. Solo alla fine lo farei girare online sul robot, per controllare tempi e integrazione con ROS.
    Come metriche: percentuale di eventi rilevati, ritardo di rilevamento, falsi allarmi per ora, matrice di confusione per transitorio/persistente e brusco/graduale, e la perturbazione più piccola che riesci a rilevare (con le masse crescenti viene abbastanza naturale).

Nella slide con e senza pinna, senza pinna la corrente di picco è più alta: mi aspettavo il contrario, mi spieghi come mai?

Nelle slide non ho trovato i risultati sulle perturbazioni sintetiche, le slide dopo "Calibration results" sono vuote.
Se riesci ad aggiungerli mi farebbe piacere vederli, anche solo un paio di esempi e una tabella riassuntiva.

# Requirements 
uv pip install -r requirements.txt

