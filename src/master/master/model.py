import torch
import torch.nn as nn

# definisce l'architettura
 
class FishStaticNet(nn.Module):
    def __init__(self, input_size=2, gru_hidden=64, mlp_hidden=128):
        # input_size: numero di canali nella sequenza 
        #gru_hidden: dim hidden state del GRU 
        #mlp _hidden: dim hidden layer dell'mlp 

        super().__init__()
 
        # Stadio 1: encoder temporale => prende la sequenza (batch, window, input_size) e 
        # restituisce l'ultimo hidden state di dim gru_hidden
        # se non basta, aumentare il numero di layers
        self.gru = nn.GRU(input_size=input_size, hidden_size=gru_hidden, num_layers=1, batch_first=True)          
 
        # Stadio 2: MLP
        # prende h(t) concatenato con [amp_des,freq_des] => dim gru_hidden+2
        self.mlp = nn.Sequential(nn.Linear(gru_hidden+2, mlp_hidden), nn.ReLU(), nn.Linear(mlp_hidden, mlp_hidden//2), nn.ReLU())
 
        # Testa 1: comando servo (theta_ref in radianti , normalizzato)
        # prende il vettore prodotto dal mlp e lo schiaccia a un singolo numero (thehta_ref)  
        ## è un nn.Linear (64,1)
        self.head_cmd = nn.Linear(mlp_hidden//2,1)
 
        # Testa 2: v_flow predetto (normalizzato)
        # serve a runtime per calcolare e_flow = v_flow_reale - v_flow_predetto
        self.head_vflow = nn.Linear(mlp_hidden//2,1)
 
    def forward(self, seq, target):
        # seq:    (batch, window, input_size) => storia sensori 
        # target: (batch, 2)  => [amp_des, freq_des] 
        # returns: 
        #   - cmd (batch,) => comando servo  
        #   - vflow (batch,) => v_flow predetto 
        #   - h (batch, gru_hidden) => contesto temporale  - verrà usato dalla rete dinamica 
        
        # GRU: prendo solo l'ultimo hidden state 
        _,h= self.gru(seq)  # si scarta il primo valore - ovvero tutti gli hidden state a ogni timestep
        h = h.squeeze(0) # rimuove la dmiensione extra ( da (num_layers, batch, gru_hidden) in (batch, gru_hidden))

        # concatenazione contesto + target desiderato
        x = torch.cat([h,target], dim=1)
        
        x = self. mlp(x)

        cmd=self.head_cmd(x).squeeze(1)
        vflow = self.head_vflow(x).squeeze(1) 

        return cmd, vflow, h 
        