# Multi-Agent Surveillance System Simulation

A coordinated surveillance system simulation implementing multiple autonomous agents for risk zone monitoring and threat detection.

## Team Members
- [Name 1]
- [Name 2]
- [Name 3]

## Project Overview
This project addresses the challenge of coordinated patrolling in urban security environments. Through a multi-agent system simulation, we demonstrate how different surveillance entities collaborate to monitor risk zones, detect threats, and coordinate responses efficiently.

The simulation focuses on solving critical challenges in open risk areas:
- Maximizing surveillance coverage
- Minimizing response times
- Ensuring infrastructure and personnel safety
- Coordinating multiple autonomous agents

## Agents Description

### Drone Agent
Primary aerial surveillance unit with capabilities:
- Real-time area monitoring
- Threat detection within defined radius
- Dynamic patrol patterns
- Coordination with security personnel
- Automatic return-to-base functionality

### Camera Agent
Stationary surveillance units with:
- Fixed position monitoring
- Wide-area coverage
- Movement detection capabilities
- Alert system integration
- Real-time feed analysis

### Security Personnel Agent
Mobile ground units featuring:
- Dynamic response capabilities
- Drone control authority
- Threat assessment
- Direct intervention abilities
- Coordination with surveillance systems

### Robber Agent (Threat)
Simulated threat entity with:
- Evasive movement patterns
- Environment awareness
- Dynamic threat behavior
- Interaction with environment

## Environment Description
- Grid-based simulation (20x20)
- Multiple terrain types
- Central landing station for drone
- Dynamic risk zones
- Real-time interaction space
- Multi-level monitoring (ground and aerial)

## Success Metrics

### 1. Threat Detection Rate (TDR)
```
TDR = Number of robbers spotted / Total number of robbers
```
Measures the system's ability to identify threats in the environment.

### 2. Area Coverage (AC)
```
AC = Number of unique grid cells visited / Total number of grid cells
```
Evaluates the effectiveness of the patrol pattern and surveillance coverage.

### 3. Chase Success Rate (CSR)
```
CSR = Number of successful chases / Number of chase attempts
```
Assesses the system's ability to maintain surveillance on detected threats.

## Communication Protocols
The system implements FIPA-compliant communication protocols for:
- Threat Detection Protocol
- Patrol Protocol
- Emergency Response Protocol
- Mission Completion Protocol

## Project Resources

### Documentation
- Technical Documentation: [Link]
- API Reference: [Link]
- Installation Guide: [Link]

### Media
- Demo Video: [Link]
- Presentation: [Link]
- Additional Resources: [Link]

### Development
- Source Code: [Link]
- Development Guide: [Link]
- Contributing Guidelines: [Link]
