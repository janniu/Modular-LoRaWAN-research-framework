# Modular LoRaWAN Research Framework

A modular and extensible simulation framework for LoRaWAN research, designed to support the evaluation of communication protocols, gateway-selection strategies, and optimization algorithms under diverse deployment environments.

The framework provides configurable models for LoRaWAN network operation, including uplink and downlink communication, gateway scheduling, radio propagation, interference, duty-cycle constraints, packet collisions, retransmissions, and energy consumption. Its modular architecture enables researchers to implement and compare different heuristic, metaheuristic, and optimization-based approaches within a consistent simulation environment.

## Features

- Modular LoRaWAN network simulator
- Configurable Industrial, Suburban, and Rural deployment scenarios
- Support for uplink and downlink communication
- RSSI, SNR, gateway load, and interference modeling
- Duty-cycle aware gateway scheduling
- Collision and capture-effect simulation
- ACK scheduling and retransmission handling
- Energy consumption modeling
- Extensible framework for heuristic and metaheuristic algorithms
- Automated statistical analysis and result generation

## Supported Algorithms

- Alg-HR
- Alg-LB
- Alg-LBHR
- E-Alg-LBHR
- CoATI
- Improved-CoATI
- Binary-CoATI
- Genetic Algorithm (GA)
- Particle Swarm Optimization (PSO)

## Performance Metrics

The framework supports the evaluation of multiple network and communication performance metrics. Depending on the objectives of a particular study, one or more of the following metrics may be analyzed:

- Packet Delivery Ratio (PDR)
- ACK Rate
- Retransmission Rate
- Communication Delay
- Throughput
- Energy Consumption
- Gateway Utilization
- Fairness Index
- Duty-Cycle Utilization

## Research Applications

This repository is intended to serve as a reusable research framework for LoRaWAN studies. Different research works may utilize different modules, algorithms, deployment scenarios, and performance metrics depending on their specific objectives. Consequently, not every study necessarily reports all supported metrics or algorithms available within this framework.

## Requirements

- Perl 5.x
- Python 3.x

## Running Simulations

```bash
./run.sh
```

## Repository Structure

- `src/` – Simulation source code
- `scripts/` – Experiment automation scripts
- `results/` – Generated simulation outputs
- `analysis/` – Statistical analysis and plotting utilities
- `docs/` – Documentation

## License

This repository is provided for academic and research purposes.
