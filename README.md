# Buildings_during_the_earthquake
Visualization of buildings vibrations during the earthquake

## Overview
This Julia-based program simulates the dynamic response of a multi-story building under earthquake-induced forces. The program integrates a physics-based model with a real-time graphical user interface (GUI), allowing the user to interactively observe how the building's structure deforms under various earthquake conditions.

The simulation is built around solving the equation of motion for a building subjected to seismic forces. The program uses:
- **GLMakie** for 3D visualization,
- **DifferentialEquations.jl** for solving the system's dynamics, and

---

## Physics Behind the Model

### Equation of Motion

The core physical model used in the simulation is governed by the equation of motion for a building under external forces, particularly those caused by an earthquake:
$`
M \ddot{x} + C \dot{x} + K x = F(t)
`$
Where:
- **M** is the mass matrix of the building,
- **C** is the damping matrix,
- **K** is the stiffness matrix,
- **F(t)** is the time-dependent force due to the earthquake.

The damping matrix **C** is calculated using **Rayleigh damping**:
$`
C = \alpha M + \beta K
`$
Where:
- $`\alpha`$ and $`\beta`$ are damping coefficients,
- **M** and **K** are the mass and stiffness matrices, respectively.

The building’s displacement is computed by solving the above second-order differential equation over time.

## Parameters and Their Effects

### 1. **Stiffness (k)**
- **Definition**: Resistance of the building to deformation under load.
- **Impact**: 
  - Higher stiffness reduces deformation.
  - Increases the natural frequency, leading to faster vibrations.
- **Formula**:  
  $` F = k \cdot \Delta x `$

### 2. **Mass (m)**
- **Definition**: The weight of the structure or material.
- **Impact**: 
  - Higher mass lowers the natural frequency, slowing vibrations.
  - Reduces sensitivity to high-frequency loads but increases inertia.
- **Formula**:  
  $`\omega_0 = \sqrt{\frac{k}{m}}`$, where $` \omega_0 `$ is the natural frequency.

### 3. **Damping (c)**
- **Definition**: The ability of the building to dissipate vibration energy.
- **Impact**: 
  - Higher damping reduces oscillation amplitudes.
  - Prevents resonance, lowering the risk of structural damage.
- **Formula**:  
  $` m \ddot{x} + c \dot{x} + k x = F(t) `$

### 4. **Height of the Building (h)**
- **Definition**: Number of floors or vertical size of the structure.
- **Impact**: 
  - Taller buildings tend to sway more due to higher leverage effects.
  - Amplifies lower-frequency vibrations (longer periods of motion).
  - Increases overall mass and changes the distribution of stiffness (less rigidity at higher points).
- **Parameter Interactions**:
  - **Stiffness**: Needs to be proportionally higher in taller buildings to maintain stability.
  - **Mass**: Increases with height, further lowering the natural frequency.
  - **Damping**: Must be sufficient to control the larger oscillations caused by greater height.

## Parameter Interactions
- **Stiffness vs. Mass**: 
  - Higher stiffness increases vibration speed; higher mass slows it.
- **Height**: Taller buildings magnify the effects of these parameters and are more sensitive to low-frequency seismic waves.
- **Damping Effect**: 
  - Critical for taller buildings to prevent excessive oscillations at the upper levels.

---

## Key Functions

### 1. `create_matrices(floors, stiffness, mass)`
Generates the mass (**M**), stiffness (**K**), and damping (**C**) matrices required for the dynamic simulation.
- **Mass Matrix**: Diagonal matrix representing the mass of each floor.
- **Stiffness Matrix**: Represents the stiffness of the building, accounting for the connections between floors.
- **Damping Matrix**: Computed using Rayleigh damping (α and β coefficients).

### 2. `dynamic_response(M, K, C, external_force_func, tspan)`
Solves the equation of motion using state-space representation, where:
- **M**: Mass matrix,
- **K**: Stiffness matrix,
- **C**: Damping matrix,
- **external_force_func**: A function representing the earthquake force at each time point.

The system is solved using the `DifferentialEquations.jl` package, and the displacement and velocity of each floor are calculated over the given time span.

### 3. `earthquake_wave(time, z, max_a, w_freq, duration)`
Defines the earthquake force as a sinusoidal function with a phase delay. This function models the propagation of seismic waves based on the height of the building.

### 4. `apply_earthquake_forcing(floors, time, max_a, w_freq, duration)`
Applies the computed earthquake forces to each floor of the building, based on the time, amplitude, frequency, and duration of the earthquake.

### 5. `sway_building!(time, floors, stiffness, mass, max_a, w_freq, duration)`
Computes the displacement of each floor at a given time, based on the previously computed mass, stiffness, and damping matrices. It also updates the building's geometry in the visualization to reflect the displacement.

### 6. `create_gui()`
Generates the graphical user interface (GUI) using **GLMakie**, allowing the user to:
- Interact with sliders to modify parameters like the number of floors, stiffness, mass, and earthquake properties.
- Visualize the building in 3D and observe real-time deformation.
- Reset parameters or simulation states.

---

## Visualization

- **3D Model**: The building is visualized as a series of floors connected by walls, with each floor represented by a rectangle. The displacement at each floor is shown in real-time as the building sways during the earthquake.
- **Interactivity**: 
  - Users can adjust parameters like the number of floors, stiffness, mass, max earthquake amplitude, frequency, and duration using sliders.
  - The GUI updates the simulation parameters in real time, and the building's geometry is deformed accordingly.
  - The program includes buttons to reset simulation parameters or restart the simulation.

---

## How to Run

### 1. **Setup**
- Save the code to a file, e.g., `earthquake_simulation.jl`.
- Ensure you have Julia installed, along with the necessary packages: `GLMakie`, `DifferentialEquations` and `LinearAlgebra`.

### 2. **Install Dependencies**
If you don't have the required packages installed, you can install them using the following commands in the Julia REPL:
```julia
using Pkg
Pkg.add("GLMakie")
Pkg.add("DifferentialEquations")
Pkg.add("LinearAlgebra")
```
### 3. Running the Simulation
Open a Julia terminal or REPL.
Navigate to the directory where the script is located.
Run the script with:
`include("earthquake_simulation.jl")`
This will open the GUI, displaying the building model.

### Sorurces of knowladges: 
- automatics lectures materials from 3 semester
- https://youtu.be/vLaFAKnaRJU?si=gtMJmBpE3IR9V5J_
- full cource https://youtube.com/playlist?list=PL48SKuieCUq81ONOaHIaWiQB8Tu7W0N0D&si=0tOTYJsylFfVchBs 
- wikipedia
- AI: ChatGPT, CloudAI
