#include <memory>
#include <morus_control/AdaGrad.h>
#include <iostream>


class probni
{
public:
    probni()
    {
    AdaGrad ja;
    mojRazred = std::unique_ptr<AdaGrad>(new AdaGrad());
    }
    
private:
    std::unique_ptr<MPC_SOLVER> mojRazred;
    
};

int main()
{
probni mojProbni;

std::cout << "Sve je proslo " << std::endl;
}
