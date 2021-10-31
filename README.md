1 Introduction


2 Model
2.1 continuous form:

ds/dt = v
M*dv/dt = - A - B * v - T_f * C * v^2 - F_e + u 

2.2 discrete form:

v_i+1^2 - v_i^2 = 2 * delta_s * a   
M * a = - A - B * v_i - T_f * C * v_i^2 - F_e + u_i 

3 Algorithm


4 Experiment


5 Start to run
    complie:
        g++ DPTest.cpp -o test
    run:
        ./test
    output:
        DPResult.txt
    visualization:
        DPVisualization.ipynb




