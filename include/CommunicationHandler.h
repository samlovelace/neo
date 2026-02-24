#ifndef COMMUNICATIONHANDLER_H
#define COMMUNICATIONHANDLER_H
 
#include <vector> 

class CommunicationHandler 
{ 
public:
    CommunicationHandler();
    ~CommunicationHandler();

    bool init(); 

    bool commandVehicle(const std::vector<double>& aVehiclePos_gl);

private:
   
};
#endif //COMMUNICATIONHANDLER_H