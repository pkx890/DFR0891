#include "DFRobot_queue.h"

static struct sReadData *cReadBufHead=NULL;
static struct sReadData *cReadBufTail = NULL;

volatile bool flag = false; //为了使不能同时入队和出队

//出队入队的时候需要上锁，保证两个任务不会操作到同一段内存

//判空
bool queue_is_empty(void)
{
    if(cReadBufHead==NULL)
        return true;
    else
        return false;
}

void queue_test(void)
{
    uint8_t data[10]={0};
    extern uint32_t esp_get_free_heap_size( void );
    while(1){
        printf("heap1:%d\r\n",esp_get_free_heap_size());
        cReadEnqueue(data,10);
        struct sReadData*p = cReadDequeue();
        free(p);
        p=NULL;
        printf("heap2:%d\r\n",esp_get_free_heap_size());
    }
}

//尾插法入队
bool cReadEnqueue(uint8_t *pbuf, uint8_t len){
    struct sReadData *p = NULL;
    p = (struct sReadData*)malloc(sizeof(struct sReadData)+len);
    if(p == NULL){
        free(p);
        p=NULL;
        return false;
    }
    p->next = NULL;
    if(cReadBufHead==NULL){
        cReadBufHead = p;
        cReadBufTail = p;
    }else{
        cReadBufTail->next = p;
        cReadBufTail = p;
    }
    p->len = len;
    p->channel = enable_channel;
    memcpy(p->data, pbuf, len);
    return true ;
}

//头出法出队
struct sReadData *cReadDequeue(void)
{
    struct sReadData *p;
    p = cReadBufHead;
    if(cReadBufHead != NULL){
        cReadBufHead = p->next;
    }else{
        //printf("EMPTY\r\n");
    }
    return p;
}

struct sReadData* getQueueTail(void)
{
    struct sReadData* p;
    p = cReadBufTail;
    return p;
}

struct sReadData *getQueueHead(void)
{
    struct sReadData *p;
    p = cReadBufHead;
    return p;
}

void clearQueue(void)
{
    while(1)
    {
        struct sReadData *p = cReadDequeue();
        if (p == NULL)
            return;
        else
        {
            free(p);
            p=NULL;
        }
    }
}