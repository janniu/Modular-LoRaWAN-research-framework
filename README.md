# Modular LoRaWAN Research Framework for Gateway Selection and Adaptive Data Rate Evaluation

This repository contains the implementation of a **modular LoRaWAN research framework** designed for the development, integration, and evaluation of gateway selection and Adaptive Data Rate (ADR) algorithms.

The framework provides configurable communication models, propagation and RSSI modelling, environmental interference, packet reception, collision and capture modelling, gateway duty-cycle management, ADR adaptation, energy modelling, and statistical performance evaluation within a unified simulation environment.

The repository also includes the implementation of the algorithms presented in the following work:

> **Multi-Objective LoRaWAN Downlink Gateway Selection Using Lightweight Heuristic and Metaheuristic Approaches**

These algorithms are integrated as representative case studies within the framework and can be evaluated under identical communication and network configurations.

---

# Key Features

## Modular Research Framework

* Configurable LoRaWAN communication models
* Gateway selection evaluation interface
* Adaptive Data Rate (ADR) support
* Environmental interference modelling
* Packet reception and collision modelling
* Capture effect implementation
* Gateway duty-cycle management
* RX1/RX2 downlink scheduling
* Energy modelling
* Statistical performance evaluation

---

## Supported Deployment Scenarios

* Industrial
* Suburban
* Rural--Agricultural

---

## Representative Gateway Selection Algorithms
* Alg-HR
* Alg-LB
* Alg-LBHR
* E-Alg-LBHR
* CoATI
* Improved-CoATI
* Binary-CoATI
* Genetic Algorithm (GA)
* Particle Swarm Optimization (PSO)

The modular architecture allows researchers to incorporate additional heuristic, metaheuristic, evolutionary, or learning-based gateway selection and ADR algorithms without modifying the underlying communication models.

---

## Performance Metrics

The framework supports comparative evaluation using

* Downlink Delay
* ACK Success Rate
* Retransmissions

---

## Requirements

* Perl (v5.10 or later)
* Python 3.x

---

# Repository Purpose

This repository accompanies the research on modular LoRaWAN simulation and gateway selection. It provides the complete framework implementation, representative optimization algorithms, experiment scripts, configuration files, and analysis tools required to reproduce the experimental results presented in the associated publications.

A lightweight configuration is provided for quick validation and functional testing. Full-scale experiments can be executed separately for comprehensive performance evaluation.

---

# Quick Start

Run a representative experiment:

```bash
./run.sh
```
