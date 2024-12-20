# Multi-Agent Surveillance System Simulation

A coordinated surveillance system simulation implementing multiple autonomous agents for risk zone monitoring and threat detection.

## Team Members
- **Diego Alejandro Calvario Aceves A01642806**  
- **Diego Rodríguez Romero A01741413**  
- **Milan Alejandro De Alba Gómez A01637667**  
- **Gonzalo Calderón Gil Leyva A01740008**

## Repository Organization
- Documentation: The Documentation folder contains all the Docs related to the Evidence, those corresponfing to the Final Evidence have Final in its name.
- Diagrams: PDFs for the class and sequence diagrams used in the documentation.
- Presentation: PDF of our final presentation.
- Graphs: Scripts and images of the Graphs that represent our sucess metrics.
- Scripts: All the scritps of our simulation (Python server and Unity scripts), those corresponfing to the Final Evidence have Final in its name.

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

## Communication Protocols
The system implements FIPA-compliant communication protocols for:
- Threat Detection Protocol
- Patrol Protocol
- Emergency Response Protocol
- Mission Completion Protocol

## Project Resources

### Documentation
- Technical Documentation: [Link](https://docs.google.com/document/d/1rUDqjiK7-KDJswpGRiRSVyGYKACJQuPOj3WiJMhQibI/edit?usp=sharing)
- Installation Guide: [Link](https://docs.google.com/document/d/1md2izcdZV1MljagRqIuXjF7du41Hkyto-3n2KENwN5k/edit?usp=sharing)

### Media
- Demo Video: [Link](https://youtu.be/W_cFJOKe2oU)
- Presentation: [Link](https://www.canva.com/design/DAGX0D9-HY0/L8idS2YYozzoWbuA-nr_Xw/edit?utm_content=DAGX0D9-HY0&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)

### Unity Project and Server File
- [Link to Drive](https://drive.google.com/drive/folders/1LwzeK-e7mb2bxrK4boZpVSBnVRqmc2fw?usp=sharing)

### Team Composition
- [Link to Drive](https://docs.google.com/document/d/1Up_nsI_3LVbfljp3XkVUJf1KqFE3RZiMI-q4zwXfATk/edit?usp=sharing)
