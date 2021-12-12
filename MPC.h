#include <gurobi_c++.h>
#include <sstream>
#include <string>
#include <math.h>
#include <vector>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

using namespace std;

float MPCCaculate(float space, float speed, vector<pair<float, float>> speedMaxInfoPart, spdlog::logger logger)
{
    logger.info("space is {}, speed is {}", space, speed);

	int part_size = speedMaxInfoPart.size();

	try{
		// Create an environment
		GRBEnv env = GRBEnv(true);
		env.set("LogFile", "./log/mpc_log.txt");
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
			obj += Kv * (v_max - V[i + 1]) * (v_max - V[i + 1]);
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

		if(model.get(GRB_IntAttr_Status) == 3)
			return U[0].get(GRB_DoubleAttr_LB);

        //print state variable

		logger.info("u0:{}, u1:{}, u2:{}, u3:{}", U[0].get(GRB_DoubleAttr_X),U[1].get(GRB_DoubleAttr_X),
		                      U[2].get(GRB_DoubleAttr_X),U[3].get(GRB_DoubleAttr_X));
        logger.info("x0:{}, x1:{}, x2:{}, x3:{}, x4:{}", X[0].getValue(),X[1].getValue(),X[2].getValue(),X[3].getValue(),X[4].getValue());
        logger.info("v0:{}, v1:{}, v2:{}, v3:{}, v4:{}", V[0].getValue(),V[1].getValue(),V[2].getValue(),V[3].getValue(),V[4].getValue());
        logger.info("a0:{}, a1:{}, a2:{}, a3:{}", a[0].getValue(),a[1].getValue(),a[2].getValue(),a[3].getValue());

		//print constraint 

		logger.info("v_max0:{}, v_max1:{}, v_max2:{}, v_max3:{}", V_max[0].getValue(),V_max[1].getValue(),V_max[2].getValue(),V_max[3].getValue(),V_max[4].getValue());
		logger.info("lb-> x0:{}, x1:{}, x2:{}, x3:{}", X_bound[0][0].getValue(),X_bound[1][0].getValue(),X_bound[2][0].getValue(),X_bound[3][0].getValue());
        logger.info("ub-> x0:{}, x1:{}, x2:{}, x3:{}", X_bound[0][1].getValue(),X_bound[1][1].getValue(),X_bound[2][1].getValue(),X_bound[3][1].getValue());
		
		return U[0].get(GRB_DoubleAttr_X);
	}
	catch (GRBException e) {
		logger.error(" Error code = {}", e.getErrorCode());
		logger.error("{}",e.getMessage());
	}
	catch (...) {
	    logger.warn(" Exception during optimization ");
    }
	return 0;
}
