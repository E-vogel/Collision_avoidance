# Collision_avoidance

This project simulates a collision avoidance and tracking algorithm where avoider agents attempt to avoid chaser agents that are trying to follow them. The simulation runs until a collision occurs or a set time limit is reached.

## Description

The simulation involves two types of agents:

- **Avoider**: These agents try to avoid collisions with the chaser agents. 
- **Chaser**: These agents try to chase the avoider agents.

The simulation space is defined within a square boundary, and both types of agents are initialized with random positions and velocities. The avoider agents take evasive actions based on the positions and velocities of the chaser agents, while the chaser agents adjust their velocities to follow the avoider agents.

## Parameters

- `AR`: Avoider range, the distance within which avoiders start taking evasive action.
- `Ak`: Avoider strength, the factor that determines how strongly the avoiders react.
- `CR`: Chaser range, the distance within which chasers start tracking the avoiders.

## Simulation Details

- **Time step**: 0.1
- **Simulation boundary**: 50 units in each direction
- **Avoider count**: 1
- **Chaser count**: 20
- **Avoider radius**: 3 units
- **Chaser initial velocity range**: [-8, 8]

## Termination Conditions

- The simulation ends if any avoider gets within a specified radius of a chaser or if the simulation time exceeds a certain limit (5000 time units).

## Usage

To run the simulation, call the function `collision_avoidance_tracking_fun(AR, Ak, CR)` with the desired parameters for avoider range, avoider strength, and chaser range.

## Example

```matlab
AR = 10;   % Avoider range
Ak = 1;    % Avoider strength
CR = 5;    % Chaser range
[steps, Avoider.Xstr, Chaser.Xstr] = collision_avoidance_tracking_fun(AR, Ak, CR);   % Outputs the time of avoidance, Avoider's coordinates, and Chaser's coordinates.
```
If you want to output animation, execute the following code.

```matlab
collision_avoidance_tracking_repeat
```

Each parameter in the code was adjusted by the genetic algorithm of the following code.

```matlab
collision_avoidance_ga
```

