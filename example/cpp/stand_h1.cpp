#include "h1_control.hpp"

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start";
    std::cin.get();
    H1Control H1Control;
    H1Control.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
