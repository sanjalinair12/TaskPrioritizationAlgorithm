<div style="margin: 40px;">




# AhToVik: Verification-Aware Resilient Orchestration for Heterogeneous Multi-Robot Systems

<p align="center">
  <img src="docs/images/ahtovik_architecture.png" alt="AhToVik Architecture" width="900"/>
</p>

## Overview

**AhToVik** is a verification-aware orchestration framework for heterogeneous multi-robot systems (HMRS) operating in dynamic and uncertain environments. The framework integrates hybrid multi-criteria decision-making, runtime monitoring, resilient recovery, and verification-aware re-orchestration into a unified lifecycle capable of maintaining mission continuity under operational disruptions.

Unlike traditional task allocation algorithms that optimize only the initial assignment, AhToVik continuously evaluates mission execution, detects failures, verifies recovery decisions, and dynamically re-orchestrates robot teams while preserving operational consistency.

The framework has been developed and evaluated using Search-and-Rescue (SAR) missions involving heterogeneous fleets consisting of UAVs, UGVs, and specialized Creeper robots.

---

# Research Contributions

The AhToVik project has evolved through three complementary research contributions.

---

## 1. Hybrid Decision-Making for Dynamic Team Formation (Core Algorithm)

The first contribution introduces the **AhToVik hybrid decision-making algorithm**, combining three complementary Multi-Criteria Decision Making (MCDM) techniques:

* Analytic Hierarchy Process (AHP)
* Technique for Order Preference by Similarity to Ideal Solution (TOPSIS)
* Modified VIKOR

The objective is to produce stable, interpretable and compromise-optimal robot rankings for heterogeneous task allocation.

### Decision-Making Pipeline

```
Mission Objectives
        │
        ▼
Criteria Definition
        │
        ▼
AHP
(Criteria Weight Calculation)
        │
        ▼
TOPSIS
(Robot Capability Evaluation)
        │
        ▼
Modified VIKOR
(Compromise Ranking)
        │
        ▼
Task Prioritization
        │
        ▼
Heterogeneous Team Formation
```

### Main Features

* Hybrid AHP-TOPSIS-VIKOR ranking
* Multi-criteria robot evaluation
* Dynamic task prioritization
* Heterogeneous team synthesis
* Compromise-based robot selection
* Scalable decision-making
* Stable ranking under uncertainty

---

## 2. Resilient Self-Healing Orchestration (RSSR)

The second contribution extends the decision-making framework into a **resilient orchestration architecture** capable of maintaining mission continuity during runtime.

Instead of treating failures as terminal events, AhToVik continuously monitors mission execution, detects disruptions, and autonomously performs recovery and re-orchestration.

### Runtime Orchestration Pipeline

```
Mission Planning
        │
        ▼
Task Allocation
        │
        ▼
Execution
        │
        ▼
Runtime Monitoring
        │
        ▼
Failure Detection
        │
        ▼
Recovery Planning
        │
        ▼
Dynamic Re-Orchestration
        │
        ▼
Mission Continuity
```

### RSSR Contributions

* Runtime monitoring
* Failure detection
* Autonomous recovery
* Dynamic task reassignment
* Self-healing orchestration
* Mission continuity preservation
* Decentralized coordination
* Asynchronous Event Bus (AEB)

The framework is evaluated against conventional recovery strategies under multiple injected failure scenarios demonstrating improved resilience and recovery performance.

---

## 3. Verification-Aware Orchestration Lifecycle (ASYDE)

The third contribution introduces a **verification-aware orchestration lifecycle**, where verification is no longer treated as an offline activity but becomes an integral component of autonomous decision making.

Verification checkpoints are embedded throughout the orchestration lifecycle to continuously validate assumptions before and during mission execution.

### Verification Lifecycle

```
Mission Objectives
        │
        ▼
Planning
        │
        ▼
Verification V1
(Feasibility Assessment)
        │
        ▼
Task Allocation
        │
        ▼
Verification V2
(Risk Ranking)
        │
        ▼
Mission Execution
        │
        ▼
Verification V3
(Runtime Monitoring)
        │
        ▼
Recovery
        │
        ▼
Verification V4
(Recovery Validation)
        │
        ▼
Re-Orchestration
```

### Verification Checkpoints

**V1 — Feasibility Verification**

* Capability verification
* Constraint validation
* Resource availability
* Initial mission feasibility

**V2 — Dynamic Risk Ranking**

* Candidate comparison
* Risk estimation
* Decision validation
* Alternative selection

**V3 — Runtime Verification**

* Battery monitoring
* Communication monitoring
* Robot health monitoring
* Task progress verification
* Assumption reassessment

**V4 — Recovery Verification**

* Recovery feasibility
* Logical consistency
* Safety verification
* Re-orchestration validation

This lifecycle enables trustworthy autonomous decision making while reducing unsafe or infeasible recovery actions.

---

# Framework Architecture

The complete AhToVik framework consists of three tightly integrated layers:

1. **Hybrid Decision-Making**

   * AHP
   * TOPSIS
   * Modified VIKOR

2. **Runtime Resilience**

   * Monitoring
   * Failure Detection
   * Recovery
   * Re-Orchestration

3. **Verification-Aware Decision Making**

   * Pre-runtime verification
   * Runtime verification
   * Recovery validation
   * Continuous assumption checking

Together these layers provide an end-to-end orchestration framework capable of resilient and verifiable autonomous operation.

---

# Features

* Hybrid AHP-TOPSIS-VIKOR decision making
* Dynamic heterogeneous team formation
* Runtime monitoring
* Failure detection
* Autonomous recovery
* Mission continuity preservation
* Verification-aware orchestration
* Risk-aware decision validation
* Decentralized coordination
* Scalable multi-robot orchestration
* ROS2 compatible architecture
* Gazebo simulation support
* Google Colab implementation

---

# Validation

AhToVik has been evaluated from three complementary perspectives.

## Decision-Making Evaluation

The hybrid ranking algorithm is compared with representative industrial and academic approaches including:

* Decentralized Prioritization Planning (DPP)
* Convex Optimization (CO)
* Learning-Based Approaches (LBA)
* Context-Aware Optimization Framework (COF)

Evaluation metrics include:

* Scalability
* Optimality
* Flexibility
* Robustness
* Adaptability
* Computational Complexity
* Interpretability
* Real-Time Performance

---

## Resilience Evaluation

The resilient orchestration framework is evaluated using heterogeneous Search-and-Rescue missions with injected failures.

Metrics include:

* Mission continuity
* Recovery success
* Recovery time
* Failure handling
* Team stability
* Reallocation efficiency
* Runtime overhead

Baselines include:

* Random Recovery
* First Available Recovery
* No Recovery

---

## Verification Evaluation

The verification-aware lifecycle is evaluated by assessing:

* Feasibility validation
* Runtime assumption checking
* Recovery correctness
* Decision consistency
* Verification overhead
* Safe re-orchestration

---

# Simulation Platforms

The framework has been implemented and evaluated using two complementary environments.

### Google Colab / Python

Used for:

* Decision-making experiments
* Algorithm benchmarking
* Ranking evaluation
* Statistical validation

---

### ROS2 + Gazebo

Used for:

* Multi-robot Search and Rescue
* Runtime monitoring
* Failure injection
* Recovery experiments
* Mission continuity evaluation
* Dynamic team reconfiguration

---

# Project Structure

```
AhToVik/
│
├── DecisionMaking/
│   ├── AHP
│   ├── TOPSIS
│   └── ModifiedVIKOR
│
├── RuntimeOrchestration/
│   ├── Monitoring
│   ├── FailureDetection
│   ├── Recovery
│   └── ReOrchestration
│
├── Verification/
│   ├── FeasibilityVerification
│   ├── RiskRanking
│   ├── RuntimeVerification
│   └── RecoveryVerification
│
├── Simulation/
│   ├── GoogleColab
│   └── Gazebo
│
├── datasets/
├── docs/
└── README.md
```

---

# Requirements

* Python 3.10+
* NumPy
* Pandas
* Matplotlib
* ROS2 (Humble or newer)
* Gazebo

Install Python dependencies:

```bash
pip install numpy pandas matplotlib
```

---

# Installation

Clone the repository:

```bash
git clone https://github.com/<username>/AhToVik.git
```

Navigate to the project directory:

```bash
cd AhToVik
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

# Usage

Run the Python implementation:

```bash
python SearchAndRescue_with_input_options.py
```

or open the provided Google Colab notebook.

For ROS2 simulation:

```bash
colcon build
source install/setup.bash
ros2 launch ahtovik simulation.launch.py
```

---

# Outputs

The framework generates:

* Criteria weights
* Robot capability rankings
* Task priorities
* Heterogeneous team formation
* Decision matrices
* TOPSIS distances
* Modified VIKOR compromise scores
* Runtime monitoring logs
* Failure detection reports
* Recovery decisions
* Verification reports
* Final mission execution statistics

---

# Future Directions

* Distributed onboard decision making
* Human-in-the-loop orchestration
* Formal runtime verification
* Digital Twin integration
* Swarm robotics
* Edge AI deployment
* Learning-assisted adaptive orchestration
* Large-scale heterogeneous robot teams

---



</div>
