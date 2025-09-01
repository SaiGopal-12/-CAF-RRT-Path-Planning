# CAF-RRT* : 2D Path Planning Algorithm Based on Circular-Arc Fillet Method

This project implements a 2D path-planning algorithm called **CAF-RRT\*** (Circular-Arc Fillet Rapidly-exploring Random Tree Star), which improves on traditional RRT* by smoothing paths using circular-arc fillets. It is designed for navigating a maze-like environment with collision-free, efficient, and visually smooth trajectories.

## 🧠 Core Idea

CAF-RRT\* enhances path planning by introducing **fillet-based smoothing** during path optimization, replacing sharp corners with smooth arcs. This results in paths that are more suitable for real-world robots by:
- Reducing abrupt turns,
- Minimizing control effort, and
- Increasing feasibility for wheeled robots.

## 📁 Project Structure

```
CAF-RRT_Path_Planning/
│
├── caf_rrt_star.py         # Main implementation of CAF-RRT* algorithm
├── maze.py                 # Defines the 2D maze environment
├── animation.py            # Handles path animation and visualization
├── solution.py             # Final solution extraction and utilities
├── running.py              # Script to run and test the full planner
├── README.md               # Project documentation (this file)
├── .gitignore              # Git ignore rules
│
├── docs/                   
│   ├── group_5_project_5.pdf
│   ├── planning_presentation.pptx
│   ├── CAF-RRT*_final_report.pdf
│   └── other docs...
```

## 🚀 How to Run

### 🛠 Requirements

- Python 3.8+
- `matplotlib`
- `numpy`

### ▶️ Execute the planner:

```bash
python running.py
```

This script will:
- Generate a maze,
- Run CAF-RRT* planner,
- Visualize the smoothed path using circular arcs.

## 🧪 Output

The planner outputs a visually smooth and collision-free path on a 2D maze. The animation will display:
- Tree expansion,
- Initial jagged path,
- Optimized smooth path using fillets.

<!-- ## 📊 Example Visualizations

> _(Add generated PNGs if desired)_ -->

## 📚 References

- Project documentation: `CAF-RRT_A_2D_Path_Planning_Algorithm_Based_on_Circular_Arc_Fillet_Method-1.pdf`
<!-- - ENPM 661 - Planning for Autonomous Robots (Fall 2024)
- RRT* base algorithm from: Karaman & Frazzoli (2011)

## 👨‍💻 Author

**Naga Venkata Siva Sai Gopal Kambhampati**  
University of Maryland, College Park  
M.Eng. in Robotics Engineering

## 📌 License

This project is for academic use. Feel free to fork, contribute, or adapt with credit. -->