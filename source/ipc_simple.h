#pragma once

#include <switch.h>
#include "sysclk/board.h"
#include "sysclk/clock_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

// Service name constant
#define SYSCLK_IPC_SERVICE_NAME "sys:clk"

// IPC command enum (from your ipc.h)
enum SysClkIpcCmd {
    SysClkIpcCmd_GetApiVersion = 0,
    SysClkIpcCmd_GetVersionString = 1,
    SysClkIpcCmd_GetCurrentContext = 2,
    SysClkIpcCmd_Exit = 3,
    SysClkIpcCmd_GetProfileCount = 4,
    SysClkIpcCmd_GetProfiles = 5,
    SysClkIpcCmd_SetProfiles = 6,
    SysClkIpcCmd_SetEnabled = 7,
    SysClkIpcCmd_SetOverride = 8,
    SysClkIpcCmd_GetConfigValues = 9,
    SysClkIpcCmd_SetConfigValues = 10,
    SysClkIpcCmd_GetFreqList = 11,
};

// IPC argument structures (from your ipc.h)
typedef struct {
    SysClkModule module;
    uint32_t hz;
} SysClkIpc_SetOverride_Args;

#ifdef __cplusplus
}
#endif