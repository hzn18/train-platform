/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:02:17 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 16:09:44
 */

#include "follow_MPC.h"

#include <gurobi_c++.h>
#include <sstream>
#include <string>
#include <math.h>
#include <vector>

#include "logger.h"
#include "constant.h"



using namespace std;

//output: 二维数组 [[x[0], v[0], u[0]], [x[1], v[1], u[1]], ... , [x[Np-1], v[Np-1], u[Np-1]]]


vector<vector<double>> FollowMPCCalculate(vector<vector<double>> predictor, double space, double speed, std::vector<std::pair<double, double>> speed_max_info_part)
{
	int part_size = speed_max_info_part.size();

	try{
		// Create an environment
		GRBEnv env = GRBEnv(true);
		env.set("LogFile", MPC_LOGGER_FILENAME);
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

        // distance between follow and leader 
        GRBLinExpr Dis[Np];

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

		for(int i = 0; i < Np; i++){
            Dis[i] = predictor[i][0] - X[i+1]; 
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
				V_max[i] += Z[i][j] * speed_max_info_part[j].second; 
			}
		}

        GRBLinExpr X_bound[Np][2];
		for(int i = 0;i < Np;i++){
			X_bound[i][0] = 0;
			X_bound[i][1] = 0;
			for(int j = 0;j < part_size - 1; j++){
				X_bound[i][0] += Z[i][j+1] * speed_max_info_part[j].first;
			}
			for(int j = 0;j < part_size - 1;j++){
				X_bound[i][1] += Z[i][j] * speed_max_info_part[j].first;
			}
			X_bound[i][1] += Z[i][part_size - 1] * 3000;
		}


		// Set objective
		//TODO£º
		double u_max = a_br * M;
		GRBQuadExpr obj = 0;
		for (int i = 0; i < Np; i++) {
			obj += K_l_v * (V[i + 1] - predictor[i][1])/v_max * (V[i + 1] - predictor[i][1])/v_max;
			obj += K_l_u * U[i] / u_max  * U[i] / u_max;
            obj += K_l_d * (1 - Dis[i]/d_des) * (1 - Dis[i]/d_des);
		}

        model.update();

		model.setObjective(obj, GRB_MINIMIZE);

		// create constraint:
		//1. power constraint
        //TODO:
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

        //5. safe distance constraint
        for(int i = 0; i < Np; i++){
            ostringstream dis_name;
			dis_name << "dis_" << i;
			model.addConstr(Dis[i] >= d_min, dis_name.str());
        }

		model.optimize();

		if(model.get(GRB_IntAttr_Status) == 3)   // 无可行解，以最保守的方式估计
			throw "infeasible";

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
		
        MPC_LOGGER.info("{}", u_info.str());
		MPC_LOGGER.info("{}", x_info.str());
        MPC_LOGGER.info("{}", v_info.str());
		MPC_LOGGER.info("{}", a_info.str());
        MPC_LOGGER.info("{}", v_max_info.str());
		MPC_LOGGER.info("{}", x_lb_info.str());
		MPC_LOGGER.info("{}", x_ub_info.str());
		
        vector<vector<double>> result;
		for(int i = 0; i < Np; i++){
			result.push_back(vector<double>{X[i+1].getValue() , V[i+1].getValue(), U[i].get(GRB_DoubleAttr_X)});
		}

		return result;
	}
	catch (GRBException e) {
		MPC_LOGGER.error(" Error code = {}", e.getErrorCode());
		MPC_LOGGER.error("{}",e.getMessage());
	}
	catch (string str){
		MPC_LOGGER.error("{}", str);
	}
	catch (...) {
	    MPC_LOGGER.warn(" Exception during optimization ");
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
