import torch
import torch.nn as nn

# H: quanti istanti passati predice la testa storia
# a 20 Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H = 20

# N_OUTPUTS: quanti canali predicono le teste.
# 2 = [sensor_diff, current]  

N_OUTPUTS = 2

# CTX_DIM: dimensione del vettore di contesto statico iniettato SOLO nell'MLP
# (non nella GRU). Ordine: [amp, freq, sin(phase), cos(phase), d_amp, d_freq].
# Deve combaciare con CTX_DIM in dataset.py.
CTX_DIM = 6


class FishSensorEstimator(nn.Module):
	def __init__(self, input_size=3, gru_hidden=512, mlp_hidden=128, h=H, ctx_dim=CTX_DIM):
		"""
		Stimatore: data una finestra temporale di comandi motore,
		predice la risposta sensoriale attesa.

		input_size:  numero di canali in ingresso (1 => storia normalizzata di
				[tail_target_rad])
		gru_hidden:  dimensione hidden state GRU
		mlp_hidden:  dimensione hidden layer MLP
		h:       quanti istanti predice la testa storia (= lunghezza finestra input)
		ctx_dim: dimensione del vettore di contesto statico (amp, freq, sin/cos
				phase, d_amp, d_freq) concatenato all'ingresso di ENTRAMBE le
				teste MLP. NON entra nella GRU: la ricorrenza resta un encoder
				della sola dinamica osservata (storia di cmd).
		"""
		super().__init__()
		self.h = h
		self.ctx_dim = ctx_dim

		# Stadio 1: GRU — encoder temporale
		self.gru = nn.GRU(
			input_size=input_size,
			hidden_size=gru_hidden,
			num_layers=1,
			batch_first=True,
		)

		# Stadio 2a: MLP per la testa storia
		# applicato a tutti gli h hidden state (batch, h, gru_hidden), con il
		# contesto (batch, h, ctx_dim) concatenato sull'ultima dim.
		# nn.Linear/nn.Sequential agiscono sull'ultima dim => ok su tensori 3D
		self.mlp_history = nn.Sequential(
			nn.Linear(gru_hidden + ctx_dim, mlp_hidden),
			nn.ReLU(),
			nn.Linear(mlp_hidden, mlp_hidden // 2),
			nn.ReLU(),
		)

		# Stadio 2b: MLP per la testa futuro
		# applicato all'ultimo hidden state h(t) (batch, gru_hidden) + contesto
		self.mlp_future = nn.Sequential(
			nn.Linear(gru_hidden + ctx_dim, mlp_hidden),
			nn.ReLU(),
			nn.Linear(mlp_hidden, mlp_hidden // 2),
			nn.ReLU(),
		)

		# Testa storia: (batch, h, mlp_hidden//2) -> (batch, h, N_OUTPUTS)
		self.head_history = nn.Linear(mlp_hidden // 2, N_OUTPUTS)

		# Testa futuro: (batch, mlp_hidden//2) -> (batch, N_OUTPUTS)
		self.head_future = nn.Linear(mlp_hidden // 2, N_OUTPUTS)


	def forward(self, seq, ctx):
		"""
		seq:     (batch, h, 1)        => storia normalizzata di [tail_target_rad]
		ctx:     (batch, ctx_dim)     => contesto statico della finestra
					[amp, freq, sin(phase), cos(phase), d_amp, d_freq]

		returns:
			pred_history  (batch, h, N_OUTPUTS)
			pred_future   (batch, N_OUTPUTS)
			h             (batch, gru_hidden)
		"""
		all_h, h_n = self.gru(seq)          # (batch, h, gru_hidden), (1, batch, gru_hidden)
		h = h_n.squeeze(0)                  # (batch, gru_hidden)

		# testa storia: MLP su tutti gli hidden state + contesto -> sensori passati.
		# il contesto e' statico per finestra: lo espando su tutti gli h timestep.
		ctx_hist = ctx.unsqueeze(1).expand(-1, self.h, -1)   # (batch, h, ctx_dim)
		x_hist = self.mlp_history(torch.cat([all_h, ctx_hist], dim=-1))  # (batch, h, mlp_hidden//2)
		pred_history = self.head_history(x_hist)         # (batch, h, N_OUTPUTS)

		# testa futuro: MLP sull'ultimo hidden state + contesto -> sensori t+1
		x_fut = self.mlp_future(torch.cat([h, ctx], dim=-1))  # (batch, mlp_hidden//2)
		pred_future = self.head_future(x_fut)            # (batch, N_OUTPUTS)

		return pred_history, pred_future, h