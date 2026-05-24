# Boids Model for Multi-Agent Simulation of Wildlife Conservation

A Python simulation of wildlife dynamics using Craig Reynolds' Boids algorithm, extended to model prey, poachers, rangers, and drones in an ecological environment.

**Paper:** [Boids Model for Multi-Agent Simulation of Wildlife Conservation](https://drive.google.com/file/d/1k9NPXKhQ_eTqY2QLu4NGCJhdaL6McMJr/view?usp=sharing)

---

## What this demonstrates

Classical agent-based modelling rarely appears in wildlife conservation analysis. This project applies the Boids flocking algorithm — typically used to simulate birds or fish — to a predator-prey-ranger ecosystem, producing emergent spatial patterns and population dynamics that reflect real-world conservation pressures.

The simulation captures:
- How prey populations respond to poaching pressure and resource availability
- The probability of rangers intercepting poachers before a prey attack
- Realistic ecological feedback loops (reproduction, overpopulation triggers, ranger reinforcement)

---

## Agents

| Symbol | Agent | Behaviour |
|---|---|---|
| White triangle | Prey | Flocks, self-reproduces, avoids poachers |
| Red triangle | Poacher | Moves toward prey; flees rangers |
| Green triangle | Ranger | Detects and intercepts poachers |
| Yellow triangle | Drone | Detects poacher coordinates; redirects rangers |
| Blue circle | Attraction point | Resource that draws prey (e.g. watering holes) |

---

## Core algorithms

Each agent navigates using three Boids steering rules:

- **Separation** — avoids collisions with nearby peers; prey also treats poachers as obstacles
- **Cohesion** — moves toward the geometric centre of its flock
- **Alignment** — matches velocity and direction of neighbouring agents

Extended with:
- **Field of vision** — each agent has a configurable view angle affecting neighbour detection
- **Obstacle avoidance** — prey detect poachers from distance and disrupt flock to escape
- **Self-reproduction** — prey replicate at defined rates; offspring placed adjacent to maintain formation
- **Dynamic population rules** — ranger count increases when poachers exceed threshold; new poachers introduced when prey overpopulate

---

## Key findings

Three case studies were run (100 simulation runs each):

1. **Attraction points increase prey survival** — prey population rose from 23.2 (0 points) to 26.4 (3 points) on average, as resources improved mobility and evasion
2. **Rangers become ineffective when outnumbered** — probability of successful interception drops below 0.10 at 4+ poachers against 3 rangers
3. **Realistic 60-day ecosystem dynamics** — prey peak around day 20, followed by periodic decline from poaching; ranger reinforcement stabilises the system over time

Population data is recorded at every timestep and saved to `species.csv`; population graphs are generated automatically at the end of each run.

---

## Setup

**Requirements:** Python 3, PyGame, PyOpenGL, Pyglet, Pandas

```bash
pip install -r requirements.txt
python -m boids
```

**Controls**

| Key | Action |
|---|---|
| `Q` | Quit |
| `A` | Adjust prey vision angle |
| `W` | Add attraction point (resource) |
| `V` | Show steering vectors |

---

## Project structure

```
boids/          # Core simulation (agents, algorithms, environment)
tests/          # Test suite
species.csv     # Population output data
requirements.txt
Makefile
```

---

## Built with

- Python (PyGame + PyOpenGL for rendering, Pyglet + Pandas for analysis)
- Boids algorithm — Craig Reynolds (1986)
- Extended from Michael Dodsworth's single-entity Boids implementation

---

## License

MIT
