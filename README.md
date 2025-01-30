# Mars Rover Path Planning Simulator

A sophisticated simulation environment for testing and visualizing autonomous rover navigation algorithms in challenging terrain conditions.


## Features

### Core Functionality
- Real-time path planning and navigation
- Multiple pathfinding algorithms (A*, Dijkstra, Energy-Efficient A*)
- Dynamic terrain generation
- Energy consumption simulation
- Interactive visualization

### Technical Highlights
- Advanced A* pathfinding with terrain cost consideration
- Real-time telemetry and analytics
- Custom terrain generation algorithms
- Comprehensive mission data collection
- Interactive PyQt5-based GUI

## Technical Architecture

```
mars-rover-simulator/
├── src/
│   ├── main.py           # Main application and GUI
│   ├── terrain.py        # Terrain generation
│   ├── pathfinding.py    # Pathfinding algorithms
│   ├── rover.py          # Rover logic and movement
│   └── telemetry.py      # Data collection and analysis
├── tests/
│   └── test_pathfinding.py  # Unit tests
├── docs/
│   ├── architecture.md      # Technical documentation
│   └── demo.gif            # Demo animation
└── README.md
```

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/mars-rover-simulator.git
cd mars-rover-simulator
```

2. Create and activate virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

## Running the Application

```bash
python src/main.py
```

## Running Tests

```bash
python -m pytest tests/
```

## Technical Details

### Pathfinding Algorithms

The simulator implements three pathfinding algorithms:

1. **A* Algorithm**
   - Traditional A* with terrain cost consideration
   - Optimal path finding with distance heuristic
   - Diagonal movement support

2. **Dijkstra's Algorithm**
   - Complete graph exploration
   - Guaranteed optimal path
   - Higher computational cost

3. **Energy-Efficient A***
   - Custom implementation prioritizing energy conservation
   - Terrain cost weighting
   - Optimized for battery efficiency

### Terrain Types and Costs

| Terrain Type | Energy Cost | Description |
|--------------|-------------|-------------|
| Clear Path   | 1x         | Basic movement cost |
| Sand         | 2x         | Moderate resistance |
| Rocks        | 3x         | High resistance |
| Obstacle     | ∞          | Impassable |

### Performance Metrics

The simulator collects and analyzes various performance metrics:

- Path length and efficiency
- Energy consumption
- Success rate
- Terrain distribution
- Completion time

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Terrain generation inspired by procedural generation techniques
- Pathfinding implementations based on classic algorithms
- UI design influenced by modern dark mode interfaces

---

## Case Study

### Problem Statement
Developing autonomous navigation systems for Mars rovers presents unique challenges due to:
- Limited energy resources
- Varied terrain conditions
- Need for efficient path planning
- Real-time decision making requirements

### Solution Approach
This simulator addresses these challenges through:
1. Advanced pathfinding algorithms considering energy efficiency
2. Realistic terrain generation
3. Comprehensive performance monitoring
4. Interactive visualization for analysis

### Results
The simulator demonstrates:
- Successful navigation in complex terrain
- Efficient energy usage through smart path planning
- Real-time performance analysis
- Intuitive visualization of rover behavior

### Future Enhancements
Planned improvements include:
- Multiple rover coordination
- Dynamic obstacle handling
- Machine learning integration
- 3D visualization

## Contact

Your Name - your.email@example.com
Project Link: [https://github.com/yourusername/mars-rover-simulator](https://github.com/yourusername/mars-rover-simulator)
