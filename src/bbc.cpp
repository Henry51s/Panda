// make this a bang-bang controller and implement it so that the function can be initialized in the main program. We will also need to make a getPressure() function so that the bbc can continuously access the most up to date pressure. 

#include <cmath>

class BangBangController {
    public:
        bool BangBangUp(double currentPressure, double targetPressure, double lower_deadband, double upper_deadband, bool currentlyOpen) {
            if (currentPressure < targetPressure - lower_deadband) {
                return true;
            }
            else if (currentPressure > targetPressure + upper_deadband) {
                return false;
            } else {
                return currentlyOpen;
            }
        }
};