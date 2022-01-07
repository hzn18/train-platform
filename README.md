## Introduction
**Train-Platform** is my graduation project which is intended both as a theory reference for scientists to understand related work and as a experiment tool for engineers to practice their controller design.
## Features
This project initially tends to study cooperative control theory and methods. Besides this requirement, I spent much time for extensive functions and now this platform includes:
- dynamics model
- controller design
- computer simulation 
- visualization 
- documentation

## Configuration
1. Gurobi

2. spdlog

After you configure these third-party, you should change address information in makefile. 

## Getting Started

1. dp

   ``` 
   make dp_ref
   ./dp_ref
2. leader mpc
   ``` 
   make leader
   ./leader
3. follow mpc
   ``` 
   make follow
   ./follow
   ```
4. data-driven mpc
   ```
   make data_driven_leader
   ./data_driven_leader
   ```

## File Structrue
```
train-platform
├─ README.md
├─ db
├─ examples
│  └─ demo
├─ include
├─ logs
├─ makefile
├─ src
│  ├─ calculate
│  │  └─ follow_MPC.cpp
│  ├─ communication
│  ├─ config
│  │  └─ logger.cpp
│  ├─ control
│  │  ├─ follow_controller.cpp
│  │  └─ predictor.cpp
│  ├─ environment
│  │  ├─ read_speed_limit.cpp
│  │  └─ read_speed_max.cpp
│  ├─ model
│  │  └─ dynamic_model.cpp
│  └─ programming
├─ user
│  ├─ analysis
│  ├─ result
│  └─ visualization
└─ utils
   └─ exception

```


## Model
### continuous form

$$
\dot{x} = v \tag{1}
$$

$$
Ma = -A - Bv-T_fCv^2-F_e+u \tag{2}
$$

### discrete form

$$
v_{i+1}^2 - v_{i}^2 = 2a_is \tag{3}
$$

$$
Ma_i = -A-Bv_i-T_fCv_i^2-F_e+u_i \tag{4}
$$


