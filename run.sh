rm ./log/*

LeaderControlfile=LeaderControlTest.cpp

g++ -m64 -g -o main ${LeaderControlfile} -I/opt/gurobi950/linux64/include/ -I/opt/spdlog-1.x/include/ -L/opt/gurobi950/linux64/lib -lgurobi_g++5.2 -lgurobi95 -lm

./main

rm main
