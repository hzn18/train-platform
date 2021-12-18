#include <gurobi_c++.h>
#include <sstream>
#include <string>
#include <math.h>
#include <vector>
#include <iostream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "../utils/exception/MyException.h"

using namespace std;

//output: 二维数组 [[x[0], v[0], u[0]], [x[1], v[1], u[1]], ... , [x[Np-1], v[Np-1], u[Np-1]]]

vector<vector<double>> LeaderMPCCaculate(double space, double speed, vector<pair<double, double>> speedMaxInfoPart, string mpc_filename ,spdlog::logger logger)
{
    logger.info("space is {}, speed is {}", space, speed);
	int part_size = speedMaxInfoPart.size();

	try{
		// Create an environment
		GRBEnv env = GRBEnv(true);
		env.set("LogFile", mpc_filename);
		env.set("LogToConsole", "0");
		env.start();// Create an empty model
		GRBModel model = GRBModel(env);
		// initial state
		double x0 = space;
		double v0 = speed;
		double a0 = 0;
		// create control variables + function limit
		GRBVar U[Np];
		for (int i = 0; i < Np; i++) {
			ostringstream vname;
			vname << "u" << i;
			U[i] = model.addVar(-M * a_br, M * a_dr, 0.0, GRB_CONTINUOUS, vname.str());
		}
		// create state equation
		GRBLinExpr X[Np + 1];
		GRBLinExpr V[Np + 1];
		GRBLinExpr a[Np + 1];

		X[0] = x0;
		V[0] = v0;
		a[0] = a0;

		// binary variable
		GRBVar Z[Np][part_size];
		for (int i = 0; i < Np; i++) {
			for (int j = 0; j < part_size;j++){
				ostringstream vname;
			    vname << "Z_" << i << "_" << j;
			    Z[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, vname.str());
			}
		}

		// dynamic equation
		for (int i = 1; i <= Np; i++) {
			a[i - 1] = (U[i - 1] - A - B * V[i - 1]) / M;
			X[i] = X[i - 1] + Ts * V[i - 1] + 0.5 * Ts * Ts * a[i - 1];
			V[i] = V[i - 1] + Ts * a[i - 1];
		}


		// v max
		GRBLinExpr V_max[Np] = {0};
        GRBLinExpr Z_sum[Np] = {0};
        for(int i = 0;i < Np;i++){
            for(int j = 0; j < part_size;j++){
                Z_sum[i] += Z[i][j];
            }
        }

        for(int i = 0;i < Np;i++){
			for(int j = 0;j < part_size;j++){
				V_max[i] += Z[i][j] * speedMaxInfoPart[j].second; 
			}
		}

        GRBLinExpr X_bound[Np][2];
		for(int i = 0;i < Np;i++){
			X_bound[i][0] = 0;
			X_bound[i][1] = 0;
			for(int j = 0;j < part_size - 1; j++){
				X_bound[i][0] += Z[i][j+1] * speedMaxInfoPart[j].first;
			}
			for(int j = 0;j < part_size - 1;j++){
				X_bound[i][1] += Z[i][j] * speedMaxInfoPart[j].first;
			}
			X_bound[i][1] += Z[i][part_size - 1] * 3000;
		}


		// Set objective
		//TODO£º
		double u_max = a_br * M;
		GRBQuadExpr obj = 0;
		for (int i = 0; i < Np; i++) {
			obj += Kv * (1 - V[i + 1]/v_max) * (1 - V[i + 1]/v_max);
			obj += Ku * U[i] / u_max  * U[i] / u_max;
		}

        model.update();

		model.setObjective(obj, GRB_MINIMIZE);

		// create constraint:
		//1. power constraint
        /*
		for (int i = 1; i <= Np; i++) {
			ostringstream p_lb_name;
			ostringstream p_ub_name;
			p_lb_name << "p_lb_" << i - 1;
			p_ub_name << "p_ub_" << i - 1;
			model.addQConstr(V[i] * U[i - 1] >= -P_br, p_lb_name.str());
			model.addQConstr(V[i] * U[i - 1] <= P_dr, p_ub_name.str());
		}
        */
		//2. speed constraint
		for (int i = 1; i <= Np; i++) {
			ostringstream v_lb_name;
			ostringstream v_ub_name;
			v_lb_name << "v_lb_" << i - 1;
			v_ub_name << "v_ub_" << i - 1;
			model.addConstr(V[i] >= 0, v_lb_name.str());
			model.addConstr(V[i] <= V_max[i-1], v_ub_name.str());
		}

		//3. M method
		for(int i = 1; i <= Np;i++){
			ostringstream x_lb_name;
			ostringstream x_ub_name;
			x_lb_name << "x_lb_" << i - 1;
			x_ub_name << "x_ub_" << i - 1;
			model.addConstr(X[i] - X_bound[i-1][0] >=0, x_lb_name.str());
			model.addConstr(X[i] - X_bound[i-1][1] <=0, x_ub_name.str());
		}
        
        //4. integer constraint
        for(int i = 0; i < Np;i++){
            ostringstream z_sum_name;
			z_sum_name << "z_sum_" << i;
			model.addConstr(Z_sum[i] == 1, z_sum_name.str());
        }

     //   model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

		model.optimize();

		logger.info("status is {}", model.get(GRB_IntAttr_Status));

		if(model.get(GRB_IntAttr_Status) == 3)   // 无可行解，以最保守的方式估计
			throw InfeasibleException();

        //print state variable

        

        ostringstream u_info;
		ostringstream x_info;
		ostringstream v_info;
		ostringstream a_info;
		ostringstream v_max_info;
		ostringstream x_lb_info;
		ostringstream x_ub_info;

        x_lb_info << "lb-> ";
		x_ub_info << "ub-> ";

		for(int i = 0; i < Np;i++){
			u_info << "u" << i << ":" << U[i].get(GRB_DoubleAttr_X) << "  ";
			x_info << "x" << i << ":" << X[i+1].getValue() << "  ";
			v_info << "v" << i << ":" << V[i+1].getValue() << "  ";
			a_info << "a" << i << ":" << a[i].getValue() << "  ";
            v_max_info << "v_max" << i << ":" << V_max[i].getValue() << "  ";
            x_lb_info << "x" << i << ":" << X_bound[i][0].getValue() << "  ";
			x_ub_info << "x" << i << ":" << X_bound[i][1].getValue() << "  ";
		}

        logger.info("{}", u_info.str());
		logger.info("{}", x_info.str());
        logger.info("{}", v_info.str());
		logger.info("{}", a_info.str());
        logger.info("{}", v_max_info.str());
		logger.info("{}", x_lb_info.str());
		logger.info("{}", x_ub_info.str());

        vector<vector<double>> result;
		for(int i = 0; i < Np; i++){
			result.push_back(vector<double>({X[i+1].getValue() , V[i+1].getValue(), U[i].get(GRB_DoubleAttr_X)}));
		}
		return result;
	}
	catch (GRBException e) {
		logger.error(" Error code = {}", e.getErrorCode());
		logger.error("{}",e.getMessage());
	}
	catch (InfeasibleException e){
		logger.error("{}", e.message());
	}
	catch (...) {
	    logger.warn(" Exception during optimization ");
    }

    // 抛出异常，以最保守方式估计
    vector<vector<double>> result;

    double function = -M * a_br;
    double v = speed;
	double s = space;
	for(int i = 0; i < Np; i++){
        double a = (function - A - B * v  - T_f_C * v * v) / M;
        v += a * Ts;
	    s += v * Ts + 0.5 * a * Ts * Ts;
		result.push_back(vector<double>({s, v, function}));
	}

	return result;
}
