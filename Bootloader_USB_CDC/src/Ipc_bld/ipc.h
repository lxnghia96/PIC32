#ifndef IPC_INTERFACES_H
#define IPC_INTERFACES_H

#include <stdbool.h>

#define ERROR                       (0U)
#define SUCCESS                     (1U)

#define SERVICE_FW_UPDATE           (5U)
#define BLD_IPC_MIN_LENGTH          (1U)    /* 1 byte SID + 2 byte DLC + IPC PAYLOAD + 1 byte CRC */
#define BLD_RESPONSE_MAX_SIZE            (10U)

#define BLD_RESPONSE_INTO_BOOT_MODE (0x11)

uint8_t Ipc_CalculateCrc(uint8_t *data, uint8_t len);
uint8_t Ipc_PackMessage(uint8_t SerId, uint8_t *rawData, uint32_t lengthOfRawData, uint8_t *ipcMes, uint32_t* lengthOfIpcMes);

#endif /* End IPC_INTERFACES_H */
