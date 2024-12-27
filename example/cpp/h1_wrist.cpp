#include "h1_wrist.hpp"

/**
 * Main Function
 */
int main(int argc, char** argv)
{
    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     H1 Damiao Example      \n\n";

    // Initialize the DDS Channel
    std::string networkInterface = argc > 1 ? argv[1] : "";
    unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);

    // Create the H1 Wrist Controller
    auto h1wrist = std::make_shared<H1WristController>();
    h1wrist->wriststate->wait_for_connection();

    while (true)
    {
        usleep(100000);
        if(h1wrist->wristcmd->trylock())
        {
            for(auto & m : h1wrist->wristcmd->msg_.cmds())
            {
                m.kp(50); m.kd(0.3);
                m.q() = 0.95 * m.q();
                m.dq(0); m.tau(0);
            }
            h1wrist->wristcmd->unlockAndPublish();
        }

        std::cout << "-- Wrist State --\n";
        std::cout << " R: " << h1wrist->wriststate->msg_.states()[0].q() << std::endl;
        std::cout << " L: " << h1wrist->wriststate->msg_.states()[1].q() << std::endl;
        std::cout << "\033[3A"; // Move cursor up 3 lines
    }

    return 0;
}