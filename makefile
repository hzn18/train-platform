VPATH = src/calculate:src/environment:src/model:src/control:src/simulation:src/programming:src/config:examples/demo
base_objects = leader_MPC.o read_speed_max.o dynamic_model.o leader_controller.o logger.o
leader_objects = leader_sim.o $(base_objects)
convoy_objects = convoy_sim.o follow_controller.o follow_MPC.o predictor.o $(base_objects)

dp: dp_sim.o read_speed_limit.o dp.o
	g++ -m64 -g -o dp dp_sim.o read_speed_limit.o dp.o
leader: $(leader_objects)
	g++ -m64 -g -o leader $(leader_objects) -L /opt/gurobi950/linux64/lib -l gurobi_g++5.2 -l gurobi95 -lm
convoy: $(convoy_objects)
	g++ -m64 -g -o convoy $(convoy_objects) -L /opt/gurobi950/linux64/lib -l gurobi_g++5.2 -l gurobi95 -lm
data_driven_leader: data_driven_leader_sim.o $(base_objects)
	g++ -m64 -g -o data_driven_leader data_driven_leader_sim.o $(base_objects) -L /opt/gurobi950/linux64/lib -l gurobi_g++5.2 -l gurobi95 -lm

read_speed_max.o: read_speed_max.cpp
	g++ -c $< -I include/environment
read_speed_limit.o: read_speed_limit.cpp
	g++ -c $< -I include/environment
dp.o: dp.cpp
	g++ -c $< -I include/config -I include/programming
dp_sim.o: dp_sim.cpp
	g++ -c $< -I include/config -I include/environment -I include/programming
dynamic_model.o: dynamic_model.cpp
	g++ -c $< -I include/config -I include/model
leader_MPC.o: leader_MPC.cpp
	g++ -c $< -I include/config -I include/calculate -I /opt/spdlog-1.x/include/ -I /opt/gurobi950/linux64/include/
leader_controller.o: leader_controller.cpp
	g++ -c $< -I include/config -I include/control -I include/calculate -I /opt/spdlog-1.x/include/
leader_sim.o: leader_sim.cpp
	g++ -c $< -I include/config -I include/environment -I include/control -I include/model -I /opt/spdlog-1.x/include/
follow_MPC.o: follow_MPC.cpp
	g++ -c $< -I include/config -I include/calculate -I /opt/spdlog-1.x/include/ -I /opt/gurobi950/linux64/include/
predictor.o: predictor.cpp
	g++ -c $< -I include/config -I include/control -I /opt/spdlog-1.x/include/
follow_controller.o: follow_controller.cpp
	g++ -c $< -I include/config -I include/control -I include/calculate -I /opt/spdlog-1.x/include/
convoy_sim.o: convoy_sim.cpp
	g++ -c $< -I include/config -I include/environment -I include/control -I include/model -I /opt/spdlog-1.x/include/
data_driven_leader_sim.o: data_driven_leader_sim.cpp
	g++ -c $< -I include/config -I include/environment -I include/control -I include/model -I /opt/spdlog-1.x/include/
logger.o: logger.cpp
	g++ -c $< -I include/config -I /opt/spdlog-1.x/include/


clean:
	-rm dp dp_sim.o read_speed_limit.o dp.o
	-rm leader $(leader_objects) convoy $(convoy_objects)
	-rm data_driven_leader data_driven_leader_sim.o


