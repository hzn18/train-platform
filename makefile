VPATH = src/calculate:src/environment:src/model:src/control:src/simulation
objects = leader_sim.o leader_MPC.o read_speed_max.o dynamic_model.o leader_controller.o 

leader: $(objects)
	g++ -m64 -g -o leader $(objects) -L /opt/gurobi950/linux64/lib -l gurobi_g++5.2 -l gurobi95 -lm
read_speed_max.o: read_speed_max.cpp
	g++ -c $< -I include/environment
dynamic_model.o: dynamic_model.cpp
	g++ -c $< -I include/config -I include/model
leader_MPC.o: leader_MPC.cpp
	g++ -c $< -I include/config -I include/calculate -I /opt/spdlog-1.x/include/ -I /opt/gurobi950/linux64/include/
leader_controller.o: leader_controller.cpp
	g++ -c $< -I include/config -I include/control -I include/calculate -I /opt/spdlog-1.x/include/
leader_sim.o: leader_sim.cpp
	g++ -c $< -I include/config -I include/environment -I include/control -I include/model -I /opt/spdlog-1.x/include/
clean:
	-rm main $(objects)