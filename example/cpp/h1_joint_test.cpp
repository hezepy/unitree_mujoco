#include "h1_joint_test.hpp"

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

    // log
    auto now = std::chrono::system_clock::now();
    auto log_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&log_time), "%Y_%m_%d_%H_%M_%S");

    fs::path log_folder = fs::current_path() / "logs" / ss.str();
    fs::create_directories(log_folder);
    fs::path log_file_name = log_folder / "log.txt";


    H1Control H1Control;
    H1Control.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
