cd ./FollowControl

rm ../log/*

Controlfile=./FollowControlTest.cpp

Header=/opt

g++ -m64 -g -o main ${Controlfile} -I ${Header}/gurobi950/linux64/include/ -I /opt/spdlog-1.x/include/ -L ${Header}/gurobi950/linux64/lib -l gurobi_g++5.2 -l gurobi95 -lm

./main

rm main
