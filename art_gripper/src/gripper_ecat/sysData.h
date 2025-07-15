#ifndef SYSDATA_H_
#define SYSDATA_H_

//* C/C++ System Headers -----------------------------------------------------*/
#include <memory>

//* Other Lib Headers --------------------------------------------------------*/
#include <thread/dtThread.h>
#include <ecrt.h>
#include <robotData.h>
typedef struct _systemData
{
    bool run = true;
    int exitCode = 0;
    int threadStart = 0;            // thread hold and order
    int dispRestart = 0;            // terminal display restart flag

    ec_master_state_t *ecMasterState;
    ec_domain_state_t *ecDomainState;
    uint8_t *pPdoList;

    dt::Thread::ThreadTimeInfo ctrlTime;
    dt::Thread::ThreadTimeInfo dispTime;

    dt::Thread::SemInfo *dispSem;

    RobotData *robotData;

} SysData;

#endif