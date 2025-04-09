# NASA Robonaut (Two-Wheeled Robot Simulation)

This project simulates the motion of a two-wheeled robot controlled by a user-defined controller. The simulation environment is built in MATLAB, providing a platform for designing, implementing, and testing control strategies for this robotic system.

## Overview

The `DesignProblem04.m` script serves as the main entry point for running the robot simulation. It requires the name of a MATLAB function that defines the control logic as its primary input. Additionally, it accepts several optional parameters to customize the simulation behavior, including team identification, data logging, movie and snapshot saving, defining initial conditions, and controlling the display. The robot navigates a predefined road loaded from a `.mat` file.

The simulation models the 3D dynamics of a two-wheeled robot, tracking its position (x, y), orientation (theta - yaw, phi - roll), roll rate (phidot), forward velocity (v), and angular velocity (w). The user-defined controller function receives sensor data (current robot state, lateral and heading errors relative to the road, wheel speeds, and road curvature) and outputs commanded torques for the right and left wheels.

**Key Features:**

* **Customizable Controller:** Users can implement their own control algorithms by creating a MATLAB function with `initControlSystem` and `runControlSystem` sub-functions.
* **Simulation Parameter Tuning:** Various optional parameters allow for flexible experimentation with different simulation scenarios.
* **Data Logging:** Simulation data can be saved to a `.mat` file for subsequent analysis.
* **Visualization:** A graphical representation of the robot and the road is displayed during the simulation.
* **Road Following:** The robot aims to follow a predefined road trajectory.
* **Flexible Initial Conditions:** Users can specify the initial state of the robot, including position, orientation, and velocities.
* **Road Definition:** The robot operates on a road loaded from a specified `.mat` file.
* **Delayed Sensor Feedback:** The controller can optionally receive delayed sensor data.

## Getting Started

1.  **MATLAB Environment:** Ensure you have MATLAB installed.
2.  **Controller Function:** Create a MATLAB function file (e.g., `MyRobotController.m`) that contains your control logic. This file must define two sub-functions: `initControlSystem` and `runControlSystem`, adhering to the input and output specifications detailed within `DesignProblem04.m`.
3.  **Road File:** Ensure a road definition file (`road.mat` by default, or a file specified by the `'roadfile'` parameter) exists in the same directory or is accessible by MATLAB.
4.  **Run the Simulation:** Execute the `DesignProblem04.m` script from the MATLAB command window, providing the name of your controller function as the first argument.

    ```matlab
    DesignProblem04('MyRobotController');
    ```

## Controller Interface

Your controller function must define the following sub-functions:

* **`initControlSystem(parameters, data)`:** This function is called once at the beginning of the simulation. It's used to initialize controller parameters and internal data structures.
    * **Inputs:**
        * `parameters`: A struct containing physical constants, simulation settings (e.g., `tStep`, `tauMax`, `roadwidth`, `symEOM`, `numEOM`, `b`, `r`).
        * `data`: A struct for storing controller-specific data.
    * **Output:**
        * `data`: The (potentially modified) data struct.

* **`runControlSystem(sensors, references, parameters, data)`:** This function is called at each time step of the simulation. It implements your control algorithm.
    * **Inputs:**
        * `sensors`: A struct containing the current state and environment information (e.g., `t`, `e_lateral`, `e_heading`, `wR`, `wL`, `r_road`).
        * `references`: An empty struct in this problem.
        * `parameters`: The same parameters struct passed to `initControlSystem`.
        * `data`: The data struct updated in `initControlSystem` and previous calls to `runControlSystem`.
    * **Outputs:**
        * `actuators`: A struct containing the commanded torques for the right (`tauR`) and left (`tauL`) wheels.
        * `data`: The (potentially modified) data struct for use in the next time step.

## Optional Parameters

You can customize the simulation using parameter-value pairs when calling `DesignProblem04`:

* `'team'`: A string specifying a team name to display on the figure window.
* `'datafile'`: A string specifying the filename for saving simulation data (e.g., `'robot_data.mat'`).
* `'moviefile'`: A string specifying the filename for saving a movie of the simulation (e.g., `'robot_movie.mp4'`).
* `'snapshotfile'`: A string specifying the filename for saving a PDF snapshot of the final simulation frame (e.g., `'robot_snap.pdf'`).
* `'controllerdatatolog'`: A cell array of strings specifying fields in `controller.data` to log in the data file (if `'datafile'` is defined).
* `'tStop'`: A positive scalar number specifying the simulation stop time (default is `150`).
* `'initial'`: A 7x1 numerical matrix `[x; z; theta; phi; phidot; v; w]` specifying the initial state of the robot. Default values with small random perturbations are used if not specified.
* `'display'`: A logical flag (`true` or `false`) to enable or disable the live simulation display (default is `true`). Pressing 'q' on the figure will also stop the simulation if display is true.
* `'seed'`: A non-negative integer to seed the random number generator for reproducible initial conditions.
* `'roadfile'`: A string specifying the filename of the road definition file (default is `'road.mat'`).

**Example with optional parameters:**

```matlab
DesignProblem04('MyRobotController', 'team', 'RoboTeam', 'datafile', 'robot_log.mat', 'tStop', 100, 'initial', [1; 0; 0.1; 0; 0; 0.5; 0], 'display', true, 'roadfile', 'my_road.mat');
```

## Understanding the Code

The `DesignProblem04.m` script orchestrates the two-wheeled robot simulation. Here's a breakdown of its key functions:

* **`SetupSimulation(process)`:**
    * Initializes the simulation environment, including setting the random number generator seed for reproducibility.
    * Defines the physical constants of the robot, such as gravity, density, dimensions, masses, moments of inertia, and maximum wheel torque.
    * Loads the equations of motion (EOMs) from the pre-calculated file `DesignProblem04_EOMs.mat`. If this file doesn't exist, it computes and saves them for future use.
    * Loads the road geometry definition from the `.mat` file specified by `process.roadfile` (default is `road.mat`).
    * Sets the initial state of the robot based on the `initial` parameter provided by the user.
    * Calculates the initial lateral (sideways) and heading (orientation) errors of the robot relative to the closest point on the road.
    * Configures the user-defined controller by:
        * Obtaining function handles to the `initControlSystem` and `runControlSystem` functions from the provided controller name.
        * Creating a `parameters` structure containing relevant simulation constants to be passed to the controller.
        * Initializing a `data` structure for the controller's internal state.
        * Handling potential errors that might occur during the controller's initialization (`initControlSystem` function).
    * Sets up a delay buffer for sensor readings if the controller specifies a delay.

* **`RunController(process, controller)`:**
    * Executes the `runControlSystem` function of the user-provided controller.
    * Passes the current sensor readings (`sensors`), reference values (which are empty in this problem), simulation parameters (`parameters`), and the controller's internal data (`data`) as inputs.
    * Receives the commanded torques for the right (`actuators.tauR`) and left (`actuators.tauL`) wheels, along with the potentially updated controller data (`data`).
    * Includes error handling to manage exceptions within the controller's `runControlSystem` function.
    * Validates that the controller returns an `actuators` structure with the correct fields (`tauR` and `tauL`).

* **`GetReferences(process)`:**
    * Currently returns an empty struct because this simulation does not define explicit time-varying reference trajectories for the robot to follow. The goal is typically to follow the road itself.

* **`GetEOM(g, Jx, Jy, Jz, Jw, mo, mw, b, r, l)`:**
    * A function that calculates the symbolic and numeric representations of the two-wheeled robot's equations of motion based on its physical parameters (gravity, moments of inertia, masses, wheel radius, and wheel separation).

* **`CreateSensors(process)`:**
    * Generates a `sensors` struct containing information about the robot's current state and its environment:
        * `t`: Current simulation time.
        * `e_lateral`: The lateral distance error from the road centerline.
        * `e_heading`: The angular error between the robot's heading and the road's tangent.
        * `wR`: Angular velocity of the right wheel.
        * `wL`: Angular velocity of the left wheel.
        * `r_road`: The signed radius of curvature of the road at the point closest to the robot.

* **`UpdateSensors(process)`:**
    * If the controller has specified a sensor delay, this function updates a buffer that stores past sensor readings.

* **`GetSensors(process)`:**
    * Retrieves the appropriate sensor data to be passed to the controller. If a delay is specified, it returns the delayed sensor readings from the buffer.

* **`Get_TandX_From_Process(process)`:**
    * A utility function that extracts the current simulation time (`t`) and the robot's state vector (`x = [x; y; theta; phi; phidot; v; w; psiR; psiL]`) from the `process` structure. This format is suitable for the `ode45` ordinary differential equation solver.

* **`GetInput(process, actuators)`:**
    * Takes the commanded wheel torques (`actuators.tauR`, `actuators.tauL`) from the controller.
    * Applies saturation to these torque values, ensuring they do not exceed the maximum torque limit (`process.tauMax`) of the robot's motors.

* **`Get_Process_From_TandX(t, x, process)`:**
    * A utility function that takes the time (`t`) and the updated state vector (`x`) returned by the `ode45` integrator and updates the corresponding fields in the `process` structure.
    * It also updates the closest point on the road to the robot, recalculates the lateral and heading errors, and checks for conditions that should stop the simulation (e.g., going off the road, dropping a payload - though payload drop isn't explicitly modeled in the core description, or reaching the end of the road).
    * Calls `UpdateSensors` to prepare the sensor data for the next control step.

* **`GetXDot(t, x, u, process)`:**
    * Defines the continuous-time dynamics of the two-wheeled robot. It takes the current time (`t`), state vector (`x`), control inputs (`u` - the wheel torques), and the `process` structure as inputs.
    * Calculates the time derivative of the state vector (`xdot`) using the pre-computed numeric equations of motion (`process.numEOM`).

* **`CheckActuators(actuators)`:**
    * A validation function that checks if the `actuators` structure returned by the controller has the expected format, specifically ensuring it contains scalar numeric fields named `tauR` and `tauL`.

* **`ZeroActuators()`:**
    * Returns a default `actuators` structure with both right and left wheel torques set to zero. This is used as a safe default if the controller encounters an error.

* **`ShouldStop(process)`:**
    * Determines whether the simulation should stop based on the value of `process.result`. A value of `0` typically indicates a failure condition (e.g., crash), and a value of `1` indicates a success condition (e.g., reaching the end of the road).

* **`UpdateFigure(process, controller, fig)`:**
    * Manages the graphical visualization of the robot and the road. It creates a new figure or updates an existing one to display the robot's position, orientation, and the road layout.
    * It also shows relevant information such as the simulation time, the status of the controller (ON or OFF), and the simulation result (if the stopping condition has been met).
    * Handles user input via the 'q' key to allow manual termination of the simulation.
    * Uses helper functions like `DrawRoad` and `DrawRobot` to render the environment and the robot.

* **`DrawRoad(road, ds)`:**
    * A helper function that takes the road definition and a sampling interval (`ds`) to draw the road centerline and its boundaries.

* **`DrawRobot(robotfig, robot, alpha)`:**
    * A helper function that takes the current robot figure handle, the robot's geometric information, and an alpha (transparency) value to draw the 3D representation of the robot.

* **`RunSimulation(process, controller)`:**
    * The main loop of the simulation.
    * Initializes the figure and sets up movie recording if a `moviefile` is specified.
    * Iteratively:
        * Updates the graphical display using `UpdateFigure`.
        * Logs simulation data using `UpdateDatalog`.
        * Records frames for a movie if enabled.
        * Checks if the stopping condition (`ShouldStop` or user interrupt) has been met.
        * Integrates the robot's dynamics for one time step using `ode45` in `UpdateProcess`.
        * Pauses to maintain real-time simulation if the display is active.
    * Handles the shutdown process by closing the movie file, saving the logged data to a `.mat` file, and saving a snapshot of the final frame as a PDF if requested.

* **`UpdateDatalog(process, controller)`:**
    * Manages the logging of simulation data.
    * Creates a `log` structure within the `process` if it doesn't exist.
    * Records process-related data (time, states) and controller-related data (initialization time, run time, sensor values, actuator commands, and optionally user-defined controller data specified by `controllerdatatolog`).

* **`UpdateProcess(process, controller)`:**
    * Performs one step of the simulation.
    * Obtains the current time and state.
    * Gets the control input from the controller using `GetInput`.
    * Integrates the equations of motion over one time step using `ode45` with the `GetXDot` function.
    * Updates the `process` structure with the new state.
    * Gets the (empty) reference values.
    * Gets the sensor readings.
    * Runs the controller to obtain the actuator commands for the next time step using `RunController`.

## Included Test Code

The end of the `DesignProblem04.m` script typically includes an example controller implementation within a function named `Controller`. This function demonstrates the required structure with `initControlSystem` and `runControlSystem` sub-functions. It provides a basic example of how to access sensor data, parameters, and the controller's internal data, and how to output actuator commands. Users can modify this example or create their own controller functions based on this template to implement their desired control strategies for the two-wheeled robot. Running the `DesignProblem04.m` script with the name of this example controller allows users to see the simulation in action and understand the basic interaction between the simulation environment and a controller.
