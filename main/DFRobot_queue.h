#ifndef DFROBOT_QUEUE_H_
#define DFROBOT_QUEUE_H_
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdio.h"

#include <pthread.h>

struct sReadData{/*����ȡ���ݶ���*/
    struct sReadData *next;
    uint8_t channel;
    uint8_t len;
    uint8_t data[];
};

extern bool cReadEnqueue(uint8_t *pbuf, uint8_t len);
extern struct sReadData *cReadDequeue(void);
extern struct sReadData *getQueueTail(void);
extern struct sReadData *getQueueHead(void);
extern uint8_t enable_channel;
extern void clearQueue(void);
extern bool queue_is_empty(void);

extern pthread_mutex_t mutex;
#endif

extern void queue_test(void);