//params: train model parameters
#define A 9155      //unit: N
#define B 633.6     //unit: Ns/m
#define T_f_C 46.84 //unit: Ns^2/m^2
#define M 160800    //unit: kg

#define a_br  0.92   //unit: m/s^2   braking
#define a_dr  0.9    //unit: m/s^2   driving
#define P_br  220000 //unit: w
#define P_dr  120000 //unit: w
#define j_max 0.98   //unit: m/s^3

//params: simluation parameters
#define delta_s 0.5 //unit: m
#define delta_v 0.01 //unit: m/s


//params: leader control 
#define Kv 0.01
#define Ku 1

//params: MPC step size
#define Np 4
#define Ts 0.1
#define v_max 30
