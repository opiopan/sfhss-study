/*
 * sfhss.h
 *
 *  Created on: 2019/01/20
 *      Author: opiopan
 */

#ifndef SFHSS_H_
#define SFHSS_H_

#include "project.h"
#include "cc2500.h"

#define SFHSS_COARSE                0
#define SFHSS_FREQ0_VAL             0xC4
#define SFHSS_CHNUM                 30
#define SFHSS_CH(ctx)               ((ctx)->ch * 6 + 16)
#define SFHSS_CALDATA(ctx)          ((ctx)->caldata[(ctx)->ch])
#define SFHSS_SET_RECEIVED(ctx)     ((ctx)->received = 1, ((ctx)->timer ? (ctx)->rtime = __HAL_TIM_GET_COUNTER((ctx)->timer) : 0))
#define SFHSS_RESET_RECEIVED(ctx)   ((ctx)->received = 0)
#define SFHSS_ISDIRTY(ctx)          ((ctx)->packetPos == 0 && (ctx)->isDirty)
#define SFHSS_RESET_DIRTY(ctx)      ((ctx)->isDirty = FALSE)

typedef enum {
    SFHSS_INIT = 0,
    SFHSS_CALIBRATED,
    SFHSS_START_BINDING,
    SFHSS_BINDING,
    SFHSS_BINDED,
    SFHSS_CONNECTING1,
    SFHSS_CONNECTING2,
    SFHSS_CONNECTED,
}SFHSS_PHASE;

typedef struct {
    CC2500CTX*          cc2500;
    TIM_HandleTypeDef*  timer;
    int                 ch;
    volatile int        received;
    int                 rtime;
    int                 ptime[2];
    int                 interval[2];
    int                 intervalSum[2];
    int                 measureCount[2];
    int                 skipCount;
    BOOL                isDirty;
    int32_t             txaddr;
    uint8_t             phase;
    uint8_t             packetPos;
    uint16_t            data[8];
    uint8_t             hopcode;
    uint8_t             caldata[SFHSS_CHNUM];
} SFHSSCTX;

void sfhss_init(SFHSSCTX* ctx, CC2500CTX* cc2500, TIM_HandleTypeDef* timer);
void sfhss_calibrate(SFHSSCTX* ctx);

void sfhss_schedule(SFHSSCTX* ctx);
void sfhss_test(SFHSSCTX* ctx);

#endif /* SFHSS_H_ */
