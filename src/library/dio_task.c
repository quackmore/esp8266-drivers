/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <quackmore-ff@yahoo.com> wrote this file.  As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy me a beer in return. Quackmore
 * ----------------------------------------------------------------------------
 */

#include "c_types.h"
#include "osapi.h"
#include "user_interface.h"
#include "mem.h"
#include "library.h"
#include "library_dio_task.h"
#include "library_do_sequence.h"
#include "library_di_sequence.h"

static os_event_t *dio_queue;

//
// this task is for decoupling the end sequence callback
// from isr routines and/or timer functions
// (that could be an isr function when hw timer is used)
//

static void ICACHE_FLASH_ATTR dio_task(os_event_t *e)
{
    switch (e->sig)
    {
    case SIG_DO_SEQ_COMPLETED:
        // calling the end of sequence callback
        ((struct do_seq *)(e->par))->end_sequence_callack(((struct do_seq *)(e->par))->end_sequence_callack_param);

        break;
    case SIG_DI_SEQ_COMPLETED:
        stop_di_sequence_timeout((struct di_seq *)(e->par));

        // calling the end of sequence callback
        ((struct di_seq *)(e->par))->end_sequence_callack(((struct di_seq *)(e->par))->end_sequence_callack_param);

        break;
    default:
        break;
    }
}

void ICACHE_FLASH_ATTR init_dio_task(void)
{
    dio_queue = (os_event_t *)call_espbot_zalloc(sizeof(os_event_t) * DIO_TASK_QUEUE_LEN);
    system_os_task(dio_task, USER_TASK_PRIO_2, dio_queue, DIO_TASK_QUEUE_LEN);
}