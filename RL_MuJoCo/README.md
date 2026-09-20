# robo1 Get-Up PPO

This directory contains a trained PPO model that enables the two-degree-of-freedom MuJoCo robot `robo1` to recover from a fallen pose and stand upright.

## Files

The following files are required to run the project:

- `robo1_getup_ppo.zip` - Trained Stable-Baselines3 PPO model
- `robo1_env.py` - Gymnasium and MuJoCo environment
- `robo1.xml` - MuJoCo model definition
- `assets/` - STL meshes referenced by `robo1.xml`
- `play_robo1_policy.py` - Plays the trained policy
- `eval_robo1_policy.py` - Evaluates recovery from each fallen pose
- `search_all_getup.py` - Searches for candidate recovery trajectories for each fallen pose and prints `BEST_SEQ` values that can be added to `getup_reference.py`
- `getup_reference.py` - Servo target-angle waypoint sequences for each fallen pose, used to generate demonstration data
- `scripted_getup.py` - Plays the recovery trajectories from `getup_reference.py` in the MuJoCo viewer
- `pretrain_robo1_from_scripted.py` - Generates observation and target-action pairs from the reference trajectories and pretrains the PPO policy
- `train_robo1.py` - Trains or fine-tunes the PPO policy
- `export_policy_header.py` - Exports a trained model as a C header for Arduino
- `requirements.txt` - Python dependencies

## Setup

From the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r RL\requirements.txt
cd RL
```

If the environment has already been created, activate it from the repository root and enter this directory:

```powershell
.\.venv\Scripts\Activate.ps1
cd RL
```

## Play

```powershell
python play_robo1_policy.py
```

To start from a specific fallen pose:

```powershell
python play_robo1_policy.py --pose roll_pos
python play_robo1_policy.py --pose roll_neg
python play_robo1_policy.py --pose pitch_pos
python play_robo1_policy.py --pose pitch_neg
```

## Evaluate

```powershell
python eval_robo1_policy.py
```

## Training

The initial policy can be pretrained using the scripted recovery demonstrations defined in `getup_reference.py`.

To search for new recovery trajectories, run:

```powershell
python search_all_getup.py
```

Copy the resulting `BEST_SEQ` or `REFERENCE_CANDIDATES` arrays into `getup_sequence_for_pose()` in `getup_reference.py`.

Pretrain the initial policy from the demonstration data:

```powershell
python pretrain_robo1_from_scripted.py
```

This generates `robo1_getup_ppo.zip`.

The policy can then be fine-tuned with PPO:

```powershell
python train_robo1.py --model-in robo1_getup_ppo.zip --timesteps 200000 --n-envs 6 --model-out robo1_getup_ppo
```

To train a PPO policy from scratch, omit `--model-in`:

```powershell
python train_robo1.py --timesteps 200000 --n-envs 6 --model-out robo1_getup_ppo
```

## Export for Arduino

Generate `policy_network.h` from the trained model:

```powershell
python export_policy_header.py robo1_getup_ppo.zip -o policy_network.h
```

Copy the generated `policy_network.h` into the Arduino sketch directory.

## Notes

- `robo1_getup_ppo.zip` depends on the environment definitions in `robo1.xml` and `robo1_env.py`.
- MuJoCo cannot load the model without the STL files in `assets/`.
- Run the playback, evaluation, training, and export scripts from the `RL` directory.
