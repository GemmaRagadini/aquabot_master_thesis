import optuna

storage = "sqlite:///tuning_results/optuna_fish.db"
study = optuna.load_study(study_name="fish_forward_phase1", storage=storage)

# Ordina tutti i trial per val loss crescente
trials_sorted = sorted(
    [t for t in study.trials if t.value is not None],
    key=lambda t: t.value
)

print("Top 3 combinazioni:")
for i, t in enumerate(trials_sorted[:3]):
    print(f"\n#{i+1}  val_loss={t.value:.4f}")
    for k, v in t.params.items():
        print(f"  {k}: {v}")