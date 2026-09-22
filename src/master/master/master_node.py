#!/usr/bin/env python3
import math
import csv
import os
import random
import rclpy
from datetime import datetime
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from std_srvs.srv import SetBool


def clamp(x: float, lo: float, hi: float):
    if x > hi:
        return hi
    elif x < lo:
        return lo
    return x


class MasterNode(Node):

    # Limiti meccanici misurati empiricamente con Dynamixel Wizard
    # Tick 2299 => +0.385 rad   Tick 2975 => +1.422 rad
    # Centro di oscillazione: (0.385 + 1.422) / 2 = 0.903 rad
    # Semiampiezza massima:   (1.422 - 0.385) / 2 = 0.519 rad
    TAIL_MIN_RAD  = 0.385
    TAIL_MAX_RAD  = 1.422
    TAIL_BIAS_RAD = 0.903
    MAX_AMP_RAD   = 0.519

    def __init__(self):
        super().__init__('master_node')
        self.get_logger().info('master_node started.')

        # PARAMETRI DELLA TRAIETTORIA
        self.declare_parameter('target_topic', '/aquabot/dynamixel/target_position')
        self.declare_parameter('sensor_topic', '/sensor_reading')
        self.declare_parameter('mode', 'amp_sweep')
        self.declare_parameter('tail_bias_rad',  self.TAIL_BIAS_RAD)
        self.declare_parameter('tail_amp_rad',   0.4)
        self.declare_parameter('tail_freq_hz',   0.5)
        self.declare_parameter('tail_min_rad',   self.TAIL_MIN_RAD)
        self.declare_parameter('tail_max_rad',   self.TAIL_MAX_RAD)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('log_rate_hz', 20.0)
        self.declare_parameter('log_dir', 'logs')

        # parametri per variazione frequenza/ampiezza
        self.declare_parameter('trial_duration_sec', 20.0)
        self.declare_parameter('freq_min_hz', 0.5)
        self.declare_parameter('freq_max_hz', 1.5)
        self.declare_parameter('amp_min_rad', 0.1)
        self.declare_parameter('amp_max_rad', self.MAX_AMP_RAD)

        # --- FEEDBACK SENSORI: SOLO DIAGNOSTICA ---
        # Durante training e test resta SEMPRE spento: non entra mai nel moto
        # generato. Serve solo per verificare a mano il funzionamento dei sensori
        # (compute_bias_offset_diagnostic()). La traiettoria e' sempre e solo
        #   theta = center + amp * sin(phase)
        # senza alcun termine di feedback.
        self.declare_parameter("feedback_enabled", False)
        self.declare_parameter("feedback_gain", 0.0)
        self.declare_parameter("feedback_alpha", 0.1)
        self.declare_parameter("feedback_max_offset", self.MAX_AMP_RAD)

        # parametri per turning (variazione sinusoidale del centro di oscillazione)
        self.declare_parameter("turning_bias_amp_rad", 0.4)
        self.declare_parameter("turning_bias_freq_hz", 0.08)

        # --- NUOVA MODALITA': random_continuous ---------------------------------
        # Invece di girare i trial una modalita' alla volta, questa modalita' fa
        # variare in modo casuale amp/freq/center e va avanti all'infinito ("per
        # tanto di fila"). Ogni 'random_hold_sec' secondi estrae nuovi target
        # casuali per ciascun parametro (ognuno nel suo range). Il TIPO di
        # transizione e' scelto PER SINGOLO parametro, cosi' puoi averne alcuni
        # netti e altri continui contemporaneamente:
        #   'step' -> salto netto al nuovo valore all'istante dell'estrazione
        #   'ramp' -> interpolazione lineare (morbida) fino al prossimo hold
        #   'walk' -> random walk: il nuovo target e' relativo al precedente,
        #             raggiunto con rampa morbida (deriva graduale)
        # Le modalita' esistenti (std, *_sweep, turning_*) restano invariate.
        self.declare_parameter('random_hold_sec', 4.0)
        # Transizione per parametro. Default 'mixed': a OGNI estrazione si tira a
        # sorte se quel cambio sara' netto o continuo, cosi' nel dataset vengono
        # fuori da soli sia cambi netti sia continui senza sceglierli a mano.
        # Puoi comunque forzare un tipo fisso: 'step' | 'ramp' | 'walk'.
        self.declare_parameter('random_amp_transition', 'mixed')     # mixed|step|ramp|walk
        self.declare_parameter('random_freq_transition', 'mixed')    # mixed|step|ramp|walk
        self.declare_parameter('random_center_transition', 'mixed')  # mixed|step|ramp|walk
        # con 'mixed', probabilita' che un dato cambio sia netto (step) invece
        # che continuo (ramp).
        self.declare_parameter('random_step_prob', 0.5)
        # limite massimo dell'offset del centro (|center - bias|). Il margine
        # sicuro effettivo e' comunque min(questo, MAX_AMP_RAD - amp_corrente),
        # cosi' bias +/- offset +/- amp resta dentro [tail_min, tail_max].
        self.declare_parameter('random_center_max_rad', 0.20)
        # seme RNG: <0 = non deterministico (default); >=0 = riproducibile
        self.declare_parameter('random_seed', -1)

        self.trial_duration = float(self.get_parameter('trial_duration_sec').value)
        self.freq_min = float(self.get_parameter('freq_min_hz').value)
        self.freq_max = float(self.get_parameter('freq_max_hz').value)
        self.amp_min = float(self.get_parameter('amp_min_rad').value)
        self.amp_max = float(self.get_parameter('amp_max_rad').value)
        self.target_topic = self.get_parameter('target_topic').value
        self.sensor_topic = self.get_parameter('sensor_topic').value
        self.mode = self.get_parameter('mode').value
        self.bias = float(self.get_parameter('tail_bias_rad').value)
        self.amp = float(self.get_parameter('tail_amp_rad').value)
        self.freq = float(self.get_parameter('tail_freq_hz').value)
        self.tail_min = float(self.get_parameter('tail_min_rad').value)
        self.tail_max = float(self.get_parameter('tail_max_rad').value)
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.log_rate = float(self.get_parameter('log_rate_hz').value)
        self.log_dir = self.get_parameter('log_dir').value

        # feedback: solo diagnostica (vedi nota sopra)
        self.feedback_enabled = bool(self.get_parameter("feedback_enabled").value)
        self.feedback_gain = float(self.get_parameter("feedback_gain").value)
        self.feedback_alpha = float(self.get_parameter("feedback_alpha").value)
        self.feedback_max_offset = float(self.get_parameter("feedback_max_offset").value)

        self.turning_bias_amp = float(self.get_parameter("turning_bias_amp_rad").value)
        self.turning_bias_freq = float(self.get_parameter("turning_bias_freq_hz").value)

        # random_continuous
        self.random_hold = float(self.get_parameter('random_hold_sec').value)
        self.random_amp_tr = str(self.get_parameter('random_amp_transition').value)
        self.random_freq_tr = str(self.get_parameter('random_freq_transition').value)
        self.random_center_tr = str(self.get_parameter('random_center_transition').value)
        self.random_step_prob = float(self.get_parameter('random_step_prob').value)
        self.random_center_max = float(self.get_parameter('random_center_max_rad').value)
        self.random_seed = int(self.get_parameter('random_seed').value)
        if self.random_seed >= 0:
            random.seed(self.random_seed)
            self.get_logger().info(f"random_continuous: seme RNG = {self.random_seed}")

        # pubblica il target della coda
        self.publisher = self.create_publisher(Float64, self.target_topic, 10)

        self.create_subscription(
            Float32MultiArray, self.sensor_topic, self.sensor_callback, 10
        )
        self.create_subscription(Float64, '/aquabot/dynamixel/present_position',
            self.position_callback, 10)
        self.create_subscription(Float64, '/aquabot/dynamixel/present_current',
            self.current_callback, 10)
        self.last_sensor = None
        self.last_sensor_time = None
        self.t0 = None
        self.latest_target = 0.0
        self.recording = False
        self.csv_file = None
        self.csv_writer = None

        self.srv_trial = self.create_service(SetBool, 'trial', self.trial_callback)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.log_counter = 0
        self.log_every = max(1, int(round(self.control_rate / self.log_rate)))

        # --- STATO DELLA DINAMICA ---
        # I tre segnali che DEFINISCONO il moto istante per istante. Ogni
        # modalita' e' solo un generatore che aggiorna questi tre; la formula
        # finale e' unica (compute_target).
        self.current_amp    = self.amp
        self.current_freq   = self.freq
        self.current_center = self.bias   # centro EFFETTIVO (bias + eventuali variazioni)

        self.phase_acc = 0.0
        self.last_control_time = None

        # --- STATO random_continuous ---
        # Ogni segmento va da (start) a (target) nell'arco di random_hold_sec.
        # coff = offset del centro rispetto a bias (center = bias + coff).
        self.rand_initialized  = False
        self.rand_seg_t0       = 0.0
        self.rand_amp_start    = self.amp
        self.rand_amp_target   = self.amp
        self.rand_freq_start   = self.freq
        self.rand_freq_target  = self.freq
        self.rand_coff_start   = 0.0
        self.rand_coff_target  = 0.0
        # transizione EFFETTIVA del segmento corrente (con 'mixed' e' sorteggiata
        # a ogni estrazione e resta fissa per tutto il segmento)
        self.rand_amp_eff   = 'ramp'
        self.rand_freq_eff  = 'ramp'
        self.rand_coff_eff  = 'ramp'

        # feedback diagnostico (mai nel moto)
        self.current_bias_offset = 0.0

        # calibrazione sensori (diagnostica)
        self.sensor_diff_offset = 0.0
        self.calibration_samples = []
        self.calibration_done = False
        self.calibration_n = 50
        self.present_position = 0.0
        self.present_current = 0.0

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'mode':
                self.mode = p.value
                self.get_logger().info(f"mode -> {self.mode}")
            elif p.name == 'tail_freq_hz':
                self.freq = float(p.value)
                self.current_freq = self.freq
            elif p.name == 'tail_amp_rad':
                self.amp = clamp(float(p.value), 0.0, self.MAX_AMP_RAD)
                self.current_amp = self.amp
            elif p.name == 'amp_min_rad':
                self.amp_min = clamp(float(p.value), 0.0, self.MAX_AMP_RAD)
            elif p.name == 'amp_max_rad':
                self.amp_max = clamp(float(p.value), 0.0, self.MAX_AMP_RAD)
            elif p.name == 'freq_min_hz':
                self.freq_min = float(p.value)
            elif p.name == 'freq_max_hz':
                self.freq_max = float(p.value)
            elif p.name == 'trial_duration_sec':
                self.trial_duration = float(p.value)
            elif p.name == 'tail_bias_rad':
                self.bias = clamp(float(p.value), self.TAIL_MIN_RAD, self.TAIL_MAX_RAD)
            elif p.name == 'feedback_enabled':
                self.feedback_enabled = bool(p.value)
            elif p.name == 'feedback_gain':
                self.feedback_gain = float(p.value)
            elif p.name == 'turning_bias_amp_rad':
                self.turning_bias_amp = float(p.value)
            elif p.name == 'turning_bias_freq_hz':
                self.turning_bias_freq = float(p.value)
            elif p.name == 'random_hold_sec':
                self.random_hold = max(1e-3, float(p.value))
            elif p.name == 'random_amp_transition':
                self.random_amp_tr = str(p.value)
            elif p.name == 'random_freq_transition':
                self.random_freq_tr = str(p.value)
            elif p.name == 'random_center_transition':
                self.random_center_tr = str(p.value)
            elif p.name == 'random_step_prob':
                self.random_step_prob = clamp(float(p.value), 0.0, 1.0)
            elif p.name == 'random_center_max_rad':
                self.random_center_max = float(p.value)
        return SetParametersResult(successful=True)

    def sensor_callback(self, msg: Float32MultiArray):
        self.last_sensor = list(msg.data)
        self.last_sensor_time = self.get_clock().now()
        if not self.calibration_done:
            if len(self.last_sensor) >= 2:
                diff = float(self.last_sensor[0]) - float(self.last_sensor[1])
                self.calibration_samples.append(diff)
                if len(self.calibration_samples) >= self.calibration_n:
                    self.sensor_diff_offset = sum(self.calibration_samples) / len(self.calibration_samples)
                    self.calibration_done = True
                    self.get_logger().info(
                        f"Calibrazione completata: sensor_diff_offset={self.sensor_diff_offset:.2f} "
                        f"(media su {self.calibration_n} campioni)"
                    )

    def trial_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            try:
                self.start_new_trial()
                response.success = True
                response.message = "Trial started: recording ON"
            except Exception as e:
                response.success = False
                response.message = f"Failed to start trial: {e}"
        else:
            self.stop_trial()
            response.success = True
            response.message = "Trial stopped: recording OFF"
        return response

    def position_callback(self, msg: Float64):
        self.present_position = msg.data

    def current_callback(self, msg: Float64):
        self.present_current = msg.data

    def start_new_trial(self):
        self.stop_trial()
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.log_dir, f"trial_{stamp}.csv")
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # --- LOG PULITO: solo le cose interpretabili ---
        # Il moto e' theta = center + amp*sin(phase); le tre colonne
        # center_rad / tail_amp_rad / tail_freq_hz lo definiscono per intero.
        self.csv_writer.writerow([
            "mode",
            "t_ros_sec",
            "t_rel_sec",
            "tail_target_rad",     # comando prodotto (theta)
            "center_rad",          # centro EFFETTIVO di oscillazione (bias + variazioni)
            "tail_amp_rad",        # ampiezza corrente
            "tail_freq_hz",        # frequenza corrente
            "phase_rad",           # fase accumulata mod 2pi (diagnostica)
            "real_position_rad",   # posizione reale del motore (diagnostica plot)
            "present_current_ma",  # corrente misurata
            "sensor_values",       # letture sensori grezze
        ])
        self.csv_file.flush()
        self.t0 = self.get_clock().now()
        self.last_control_time = self.t0
        self.phase_acc = 0.0
        self.current_center = self.bias
        self.current_bias_offset = 0.0
        self.rand_initialized = False   # ri-inizializza random_continuous a ogni trial
        self.recording = True
        self.get_logger().info(f"Started trial -> {filename}")

    def stop_trial(self):
        self.recording = False
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None
        self.last_control_time = None
        self.phase_acc = 0.0

        # ritorno al centro base, se ROS e' ancora attivo
        try:
            if rclpy.ok():
                msg = Float64()
                msg.data = float(clamp(self.bias, self.tail_min, self.tail_max))
                self.publisher.publish(msg)
        except Exception:
            pass

    def control_step(self):
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        if self.last_control_time is None:
            self.last_control_time = now

        t_rel = (now - self.t0).nanoseconds * 1e-9
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now

        target = self.compute_target(t_rel, dt)
        self.latest_target = target

        msg = Float64()
        msg.data = float(target)
        self.publisher.publish(msg)

        self.log_counter += 1
        if self.log_counter >= self.log_every:
            self.log_counter = 0
            self.log_step()

    # ------------------------------------------------------------------
    #  FORMULA UNICA DEL MOTO
    # ------------------------------------------------------------------
    def compute_target(self, t_rel: float, dt: float):
        """Aggiorna (amp, freq, center) secondo la modalita', poi applica
        l'UNICA formula del moto:  theta = center + amp * sin(phase_acc).

        Nessun ramo di feedback: il feedback sensori e' solo diagnostico e non
        entra mai qui (vedi compute_bias_offset_diagnostic)."""
        # 1) il generatore di modalita' aggiorna amp/freq/center e fa avanzare la fase
        self._update_mode(t_rel, dt)

        # 2) formula unica
        theta = self.current_center + self.current_amp * math.sin(self.phase_acc)
        return clamp(theta, self.tail_min, self.tail_max)

    def _advance_phase(self, dt: float):
        self.phase_acc += 2.0 * math.pi * self.current_freq * dt

    def _update_mode(self, t_rel: float, dt: float):
        """Ogni modalita' e' SOLO un generatore di (amp, freq, center).
        La riga del moto non e' piu' duplicata: sta in compute_target."""
        mode = self.mode

        if mode == 'std':
            self.current_amp = self.amp
            self.current_freq = self.freq
            self.current_center = self.bias
            self._advance_phase(dt)

        elif mode == 'freq_sweep':
            alpha = self.triangular_profile(t_rel, self.trial_duration)
            self.current_freq = self.freq_min + alpha * (self.freq_max - self.freq_min)
            self.current_amp = self.amp
            self.current_center = self.bias
            self._advance_phase(dt)

        elif mode == 'amp_sweep':
            alpha = self.triangular_profile(t_rel, self.trial_duration)
            self.current_amp = self.amp_min + alpha * (self.amp_max - self.amp_min)
            self.current_freq = self.freq
            self.current_center = self.bias
            self._advance_phase(dt)

        elif mode == 'combined_sweep':
            PHI = 1.6180339887
            alpha_amp  = self.triangular_profile(t_rel, self.trial_duration)
            alpha_freq = self.triangular_profile(t_rel, self.trial_duration / PHI)
            self.current_amp  = self.amp_min  + alpha_amp  * (self.amp_max  - self.amp_min)
            self.current_freq = self.freq_min + alpha_freq * (self.freq_max - self.freq_min)
            self.current_center = self.bias
            self._advance_phase(dt)

        elif mode == 'turning_combined':
            PHI = 1.6180339887
            alpha_amp  = self.triangular_profile(t_rel, self.trial_duration)
            alpha_freq = self.triangular_profile(t_rel, self.trial_duration / PHI)
            self.current_amp  = self.amp_min  + alpha_amp  * (self.amp_max  - self.amp_min)
            self.current_freq = self.freq_min + alpha_freq * (self.freq_max - self.freq_min)
            # il centro oscilla lentamente: ORA e' loggato nel center effettivo
            self.current_center = self.bias + self.turning_bias_amp * math.sin(
                2.0 * math.pi * self.turning_bias_freq * t_rel)
            self._advance_phase(dt)

        elif mode == 'random_continuous':
            # variazione casuale continua di amp/freq/center (vedi metodi sotto)
            self._update_random_continuous(t_rel, dt)
            self._advance_phase(dt)   # fase continua: nessuno scalino sul segnale

        else:
            # fallback: fermo al centro
            self.current_amp = 0.0
            self.current_center = self.bias

    def triangular_profile(self, t_rel: float, duration: float) -> float:
        if duration <= 0.0:
            return 0.0
        tau = (t_rel % duration) / duration
        if tau < 0.5:
            return 2.0 * tau
        else:
            return 2.0 * (1.0 - tau)

    # ------------------------------------------------------------------
    #  MODALITA' random_continuous — variazione casuale, gira all'infinito
    # ------------------------------------------------------------------
    def _draw_target(self, transition: str, lo: float, hi: float,
                     prev_target: float) -> float:
        """Estrae il nuovo target di un parametro nel range [lo, hi].
        'walk' = random walk: si sposta rispetto al target precedente (deriva);
        'step'/'ramp' = valore assoluto uniforme nel range."""
        if hi <= lo:
            return lo
        if transition == 'walk':
            # passo casuale pari al massimo a meta' del range, poi clamp
            delta = random.uniform(-0.5, 0.5) * (hi - lo)
            return clamp(prev_target + delta, lo, hi)
        return random.uniform(lo, hi)

    def _seg_value(self, transition: str, start: float, target: float,
                   alpha: float) -> float:
        """Valore del parametro dentro il segmento corrente.
        'step' -> salto netto (sta sul target per tutto il segmento);
        'ramp'/'walk' -> interpolazione lineare morbida start->target."""
        if transition == 'step':
            return target
        return start + (target - start) * alpha   # ramp / walk

    def _resolve_transition(self, transition: str) -> str:
        """Risolve la transizione effettiva di questo segmento.
        'mixed' -> sorteggio netto/continuo (step con prob random_step_prob,
        altrimenti ramp); un tipo fisso resta invariato."""
        if transition == 'mixed':
            return 'step' if random.random() < self.random_step_prob else 'ramp'
        return transition

    def _draw_random_segment(self, t_rel: float):
        """Apre un nuovo segmento: sorteggia la transizione (netta/continua) e
        nuovi target per amp/freq/center-offset. Il range del center-offset si
        stringe in base all'amp target, cosi' bias +/- offset +/- amp resta
        dentro [tail_min, tail_max]."""
        self.rand_seg_t0 = t_rel
        # 1) tipo di cambio per questo segmento (netto o continuo, per parametro)
        self.rand_amp_eff  = self._resolve_transition(self.random_amp_tr)
        self.rand_freq_eff = self._resolve_transition(self.random_freq_tr)
        self.rand_coff_eff = self._resolve_transition(self.random_center_tr)
        # 2) nuovi target
        self.rand_amp_target = clamp(
            self._draw_target(self.rand_amp_eff, self.amp_min, self.amp_max,
                              self.rand_amp_target),
            0.0, self.MAX_AMP_RAD)
        self.rand_freq_target = self._draw_target(
            self.rand_freq_eff, self.freq_min, self.freq_max,
            self.rand_freq_target)
        # margine sicuro per l'offset del centro dato l'amp target
        margin = max(0.0, self.MAX_AMP_RAD - self.rand_amp_target)
        cmax = min(max(0.0, self.random_center_max), margin)
        self.rand_coff_target = self._draw_target(
            self.rand_coff_eff, -cmax, cmax, self.rand_coff_target)

    def _update_random_continuous(self, t_rel: float, dt: float):
        """Aggiorna (amp, freq, center) in modo casuale e continuo.
        La transizione e' scelta per singolo parametro (step/ramp/walk), quindi
        alcuni parametri possono cambiare netti e altri in modo continuo."""
        # prima entrata (o nuovo trial): parti dai valori correnti e pesca subito
        if not self.rand_initialized:
            self.rand_amp_start   = self.current_amp
            self.rand_freq_start  = self.current_freq
            self.rand_coff_start  = self.current_center - self.bias
            self.rand_amp_target  = self.rand_amp_start
            self.rand_freq_target = self.rand_freq_start
            self.rand_coff_target = self.rand_coff_start
            self.rand_initialized = True
            self._draw_random_segment(t_rel)

        hold = max(1e-3, self.random_hold)

        # fine segmento -> il valore attuale diventa lo start e si pesca il nuovo
        if (t_rel - self.rand_seg_t0) >= hold:
            self.rand_amp_start  = self.current_amp
            self.rand_freq_start = self.current_freq
            self.rand_coff_start = self.current_center - self.bias
            self._draw_random_segment(t_rel)

        alpha = clamp((t_rel - self.rand_seg_t0) / hold, 0.0, 1.0)
        self.current_amp = clamp(
            self._seg_value(self.rand_amp_eff, self.rand_amp_start,
                            self.rand_amp_target, alpha),
            0.0, self.MAX_AMP_RAD)
        self.current_freq = self._seg_value(
            self.rand_freq_eff, self.rand_freq_start,
            self.rand_freq_target, alpha)
        coff = self._seg_value(
            self.rand_coff_eff, self.rand_coff_start,
            self.rand_coff_target, alpha)
        # vincolo di sicurezza LIVE: |offset| + amp_corrente <= MAX_AMP, sempre.
        # Cosi' theta = center + amp*sin(...) resta in [tail_min, tail_max] senza
        # dover tosare la sinusoide col clamp finale (che falserebbe il log).
        cap = max(0.0, self.MAX_AMP_RAD - self.current_amp)
        coff = clamp(coff, -cap, cap)
        self.current_center = self.bias + coff

    # ------------------------------------------------------------------
    #  FEEDBACK SENSORI — SOLO DIAGNOSTICA (mai nel moto)
    # ------------------------------------------------------------------
    def compute_bias_offset_diagnostic(self):
        """Calcolo dell'offset da feedback sensoriale. NON viene chiamato da
        compute_target: durante training e test il feedback resta spento e non
        deve influenzare il moto. Tenuto solo per ispezionare a mano la
        risposta dei sensori quando serve."""
        if not self.feedback_enabled:
            self.current_bias_offset = 0.0
            return 0.0
        if self.last_sensor is None or len(self.last_sensor) < 2:
            self.current_bias_offset = 0.0
            return 0.0
        sensor_diff = float(self.last_sensor[0]) - float(self.last_sensor[1])
        sensor_diff_calibrated = sensor_diff - self.sensor_diff_offset
        target_offset = self.feedback_gain * sensor_diff_calibrated
        self.current_bias_offset = (
            (1.0 - self.feedback_alpha) * self.current_bias_offset
            + self.feedback_alpha * target_offset)
        self.current_bias_offset = clamp(
            self.current_bias_offset, -self.feedback_max_offset, self.feedback_max_offset)
        return self.current_bias_offset

    def log_step(self):
        if not self.recording or self.csv_writer is None:
            return
        now = self.get_clock().now()
        t_ros_sec = now.nanoseconds * 1e-9
        t_rel = (now - self.t0).nanoseconds * 1e-9 if self.t0 else 0.0
        phase = self.phase_acc % (2.0 * math.pi)

        if self.last_sensor is None:
            sensor_values = []
        else:
            sensor_values = self.last_sensor

        self.csv_writer.writerow([
            self.mode,
            t_ros_sec,
            t_rel,
            float(self.latest_target),
            float(self.current_center),
            float(self.current_amp),
            float(self.current_freq),
            float(phase),
            float(self.present_position),
            float(self.present_current),
            sensor_values,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.stop_trial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()