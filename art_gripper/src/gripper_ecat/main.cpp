//* C/C++ System Headers -----------------------------------------------------*/
#include <signal.h>
#include <memory>

//* Project Headers ----------------------------------------------------------*/
#include "sysData.h"
#include "robotData.h"
#include "thread/threadMan.h"
#include "device/deviceMan.h"
#include "util/dtTerm.h"
#include "gripper_ecat.h"
#include "rclcpp/rclcpp.hpp"


// Global instance of RobotData
std::shared_ptr<RobotData> g_robot_data = std::make_shared<RobotData>();
// Global instance of SysData
std::shared_ptr<SysData> g_sys_data = std::make_shared<SysData>();

static void CatchSignal(int sig)
{
    g_sys_data->run = false;
    // rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    signal(SIGTERM, CatchSignal); // kill command
    signal(SIGINT, CatchSignal);  // keyboard interrupt, Ctrl + c

    g_sys_data->robotData = g_robot_data.get();

    // dtTerm::SetupTerminal(false); // show cursor - false
    // dtTerm::ClearDisp();

    if (initDevice(g_sys_data.get())) goto error;
    if (initThread(g_sys_data.get())) goto error;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperEcat>(g_sys_data));
    rclcpp::shutdown();

     goto exitSuccess;

error:
    // dtTerm::Printf("Init Failed!!\n");  

exitSuccess:
    closeThread();
    closeDevice(g_sys_data.get());
    // dtTerm::RestoreTerminal();

    return 0;
}