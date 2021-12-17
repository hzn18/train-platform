//#include "FollowControl.h"
#include "../utils/exception/MyException.h"

int main(){
    try{
        throw InfeasibleException();
    }
    catch(InfeasibleException e){
        cout << e.message();
    }
    catch(...){
        cout << "Hello World!";
    }
    return 0;
}
