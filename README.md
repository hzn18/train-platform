## Introduction



## Getting Started

1. complie

   ``` 
   g++ DPTest.cpp -o test

2. run

   ```
   .\test

3. output

   ``` 
   DPResult.txt

4. visualization

   ``` 
   DPVisualization.ipynb
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

## Algorithm



## Experiment

![SpeedLimitCurve](.\analysis\DPSpeedVSLimitCurve.png)



