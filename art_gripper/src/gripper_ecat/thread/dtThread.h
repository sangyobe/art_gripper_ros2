#ifndef DTTHREAD_HH
#define DTTHREAD_HH

#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
namespace dt
{
namespace Thread
{
    using dt_thread_t = pthread_t;
    using dt_mutex_t = pthread_mutex_t;
    using dt_sem_t = sem_t;

/**
 * Data structure to hold runtime statistics of a thread such as period and etc.
 */
typedef struct _threadTimeInfo
    {
        double targetPeriod_ms = 0;
        double period_ms = 0;
        double algo_ms = 0;
        double algoAvg_ms = 0;
        double algoMax_ms = 0;
        int overrun = 0;
} ThreadTimeInfo;

/**
 * Data structure to hold information of a thread created by dt::Thread.
 */
typedef struct _threadInfo
{
    const char *name = nullptr;
    void *(*procFunc)(void *arg) = nullptr;
    void *procFuncArg = nullptr;
    int cpuIdx = 0;
    int priority = 0;
    size_t stackSz = 0;
    dt_thread_t id = 0;
    int listIdx = 0;
} ThreadInfo;

/**
 * Information of semaphore created.
 */
typedef struct _semInfo
{
    const char *name = nullptr;
    dt_sem_t sem;
    int listIdx = 0;
} SemInfo;

int CreateNonRtThread(ThreadInfo &thread);
int DeleteThread(ThreadInfo &thread);
int DeleteAllThread();




int CreateSemaphore(SemInfo &semInfo, unsigned int initValue = 0);
inline int PostSemaphore(SemInfo &semInfo)
{
    return sem_post(&semInfo.sem);
}

void PostAllSemaphore();


inline int WaitSemaphore(SemInfo &semInfo)
{
    return sem_wait(&semInfo.sem);
}


int DeleteSemaphore(SemInfo &semInfo);
int DeleteAllSemaphore();
 
inline void SleepForMillis(unsigned int milliseconds)
{
    usleep(milliseconds * 1E3);
}


} // namespace Thread
} // namespace dt




#endif