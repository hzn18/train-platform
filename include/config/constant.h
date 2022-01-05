/*
 * @Author: houzhinan 
 * @Date: 2022-01-04 15:52:59 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-05 12:45:44
 */

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

// params: simulation parameters
#define T 0.1

// params: dynamic programming
#define delta_s 0.5 //unit: m
#define delta_v 0.01 //unit: m/s
#define delta_s_model 0.5 //one model need this parameter
#define K_dp_v 1
#define K_dp_u 0.08

// params: leader control 
#define KV 1
#define KU 0.01
#define NP_ 8
#define TS 0.1

#define KV_DATA_DRIVEN 1
#define KU_DATA_DRIVEN 0
#define NP_DATA_DRIVEN 8
#define Ts_DATA_DRIVEN 0.1

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