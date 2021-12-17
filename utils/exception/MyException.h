#include <iostream>
#include <exception>
#include <string>

using namespace std;

class InfeasibleException: public exception{
    public:
    const string message() const throw ()
    {
        return "this optimization problem is infeasible";
    }
};

