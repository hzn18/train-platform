#ifndef CONSTANT_H
#define CONSTANT_H

// params: train model parameters
#define A 9155      //unit: N
#define B 633.6     //unit: Ns/m
#define T_f_C 46.84 //unit: Ns^2/m^2
#define M 160800    //unit: kg

#define a_br  0.92   //unit: m/s^2   braking
#define a_dr  0.9    //unit: m/s^2   driving
#define P_br  220000 //unit: w
#define P_dr  120000 //unit: w
#define j_max 0.98   //unit: m/s^3

// params: simluation parameters
#define delta_s 0.5 //unit: m
#define delta_v 0.01 //unit: m/s
#define delta_s_model 0.5 //one model need this parameter

// params: dynamic programming
#define K_dp_v 1
#define K_dp_u 0.08

// params: leader control 
#define Kv 1
#define Ku 0

// params: MPC step size
#define Np 8
#define Ts 0.1
#define v_max 30

// params: convoy control
#define train_num 2
#define d_des 30   //unit: m   safe distance
#define d_min 20   //unit: m   space policy

#define K_l_v 1
#define K_l_u 0.08

#define K_f_v 1
#define K_f_d 0.8
#define K_f_u 0.004

#endif