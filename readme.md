
# Particle-Based Target Localization

This repository contains a C++ implementation for particle-based target localization.

## Installation

### Prerequisites

*   A C++ compiler (e.g., g++, clang++)
*   CMake (version 3.15 or higher)
*   Make

### Build Instructions

1.  Clone the repository:

    ```bash
    git clone https://github.com/Kwoky-SWUST/particle-based-target-localization.git
    cd particle-based-target-localization
    ```

2.  Use catkin_make:

    ```bash
    catkin_make
    ```

## Key Features

*   **Particle Filter Implementation:** Utilizes a particle filter for robust target localization.
*   **Modular Design:**  The code is organized into modules for drone localization, filtering, graphics, and data parsing (nlink parser).
*   **Extensible Architecture:** Designed to be easily extended with new sensor models and localization algorithms.
*   **GNUplot Integration:** Includes modules for generating plots using GNUplot.
