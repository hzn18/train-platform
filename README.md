## Introduction



## Getting Started

1. dp

   ``` 
   make dp
   ./dp
2. leader mpc
   ``` 
   make leader
   ./leader
3. follow mpc
   ``` 
   make follow
   ./follow
4. visualization

   ``` 
   DPVisualization.ipynb
   ```

## File Structrue
```
train-platform
├─ README.md
├─ db
├─ examples
│  └─ demo
│     ├─ convoy_sim.cpp
│     ├─ dp_sim.cpp
│     └─ leader_sim.cpp
├─ include
│  ├─ calculate
│  ├─ communication
│  ├─ config
│  ├─ control
│  ├─ environment
│  └─ model
├─ logs
├─ makefile
├─ src
│  ├─ calculate
│  ├─ communication
│  ├─ config
│  ├─ control
│  ├─ environment
│  └─ model
├─ user
│  ├─ result
│  └─ visualization.ipynb
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

