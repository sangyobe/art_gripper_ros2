#ifndef MASTER_H
#define MASTER_H

#include <ecrt.h>
#include "sysData.h"

#define PERIOD_NS (1000000)
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)


int8_t ecMasterInit(SysData *sysData);
int8_t ecSetSlavePdo(uint16_t alias, uint16_t slavePos, uint32_t vid, uint32_t pid, const ec_sync_info_t syncs[]);
int8_t ecRegDomainPdoEntry(uint16_t index, uint8_t subindex, int *offset);
int8_t ecStart(SysData *sysData);
int8_t ecMasterM2S();
int8_t ecMasterS2M();

extern ec_master_t *master;
extern ec_domain_t *domain;

#endif