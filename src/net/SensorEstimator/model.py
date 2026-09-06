import torch.nn as nn

# H: quanti istanti passati predice la testa storia
# a 20 Hz, 20 timestep = 1 secondo = un ciclo completo a 1 Hz
H = 20

# N_OUTPUTS: quanti canali predicono le teste.
# 2 = [sensor_diff, current]  

N_OUTPUTS = 2


class FishSensorEstimator(nn.Module):
	def __init__(self, input_size=3, gru_hidden=512, mlp_hidden=128, h=H):
		"""
		Stimatore: data una finestra temporale di comandi motore,
		predice la risposta sensoriale attesa.

		input_size:  numero di canali in ingresso (1 => storia normalizzata di
				[tail_target_rad])
		gru_hidden:  dimensione hidden state GRU
		mlp_hidden:  dimensione hidden layer MLP
		h:       quanti istanti predice la testa storia (= lunghezza finestra input)
		"""
		super().__init__()
		self.h = h

		# Stadio 1: GRU — encoder temporale
		self.gru = nn.GRU(
			input_size=input_size,
			hidden_size=gru_hidden,
			num_layers=1,
			batch_first=True,
		)

		# Stadio 2a: MLP per la testa storia
		# applicato a tutti gli h hidden state (batch, h, gru_hidden)
		# nn.Linear/nn.Sequential agiscono sull'ultima dim => ok su tensori 3D
		self.mlp_history = nn.Sequential(
			nn.Linear(gru_hidden, mlp_hidden),
			nn.ReLU(),
			nn.Linear(mlp_hidden, mlp_hidden // 2),
			nn.ReLU(),
		)

		# Stadio 2b: MLP per la testa futuro
		# applicato all'ultimo hidden state h(t) (batch, gru_hidden)
		self.mlp_future = nn.Sequential(
			nn.Linear(gru_hidden, mlp_hidden),
			nn.ReLU(),
			nn.Linear(mlp_hidden, mlp_hidden // 2),
			nn.ReLU(),
		)

		# Testa storia: (batch, h, mlp_hidden//2) -> (batch, h, N_OUTPUTS)
		self.head_history = nn.Linear(mlp_hidden // 2, N_OUTPUTS)

		# Testa futuro: (batch, mlp_hidden//2) -> (batch, N_OUTPUTS)
		self.head_future = nn.Linear(mlp_hidden // 2, N_OUTPUTS)


	def forward(self, seq):
		"""
		seq:     (batch, h, 1)   => storia normalizzata di [tail_target_rad]

		returns:
			pred_history  (batch, h, N_OUTPUTS)
			pred_future   (batch, N_OUTPUTS)
			h             (batch, gru_hidden)
		"""
		all_h, h_n = self.gru(seq)          # (batch, h, gru_hidden), (1, batch, gru_hidden)
		h = h_n.squeeze(0)                  # (batch, gru_hidden)

		# testa storia: MLP su tutti gli hidden state -> sensori passati
		x_hist = self.mlp_history(all_h)                 # (batch, h, mlp_hidden//2)
		pred_history = self.head_history(x_hist)         # (batch, h, N_OUTPUTS)

		# testa futuro: MLP sull'ultimo hidden state -> sensori t+1
		x_fut = self.mlp_future(h)                        # (batch, mlp_hidden//2)
		pred_future = self.head_future(x_fut)            # (batch, N_OUTPUTS)

		return pred_history, pred_future, h