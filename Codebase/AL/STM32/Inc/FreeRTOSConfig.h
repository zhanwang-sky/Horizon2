//
//  FreeRTOSConfig.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/23.
//

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// Includes
#include "bsp_config.h"

// Definitions
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      SystemCoreClock
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configUSE_MINI_LIST_ITEM                1
#define configSTACK_DEPTH_TYPE                  uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t
#define configHEAP_CLEAR_MEMORY_ON_FREE         1
#define configUSE_APPLICATION_TASK_TAG          0

// Memory allocation related definitions.
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t) (10 * 1024))

// Hook function related definitions.
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#if defined(DEBUG)
#define configCHECK_FOR_STACK_OVERFLOW          2
#endif
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configUSE_SB_COMPLETED_CALLBACK         0

// Run time and task stats gathering related definitions.
#if defined(DEBUG)
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#endif

// Co-routine related definitions.
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

// Software timer related definitions.
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               3
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE

// The lowest interrupt priority that can be used in a call to a "set priority" function.
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      SYSTICK_INT_PRIORITY

// The highest interrupt priority that can be used by any interrupt service
// routine that makes calls to interrupt safe FreeRTOS API functions.
// DO NOT CALL INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT
// HAS A HIGHER PRIORITY THAN THIS! (higher priorities are lower numeric values.
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (TICK_INT_PRIORITY + 1UL)

// Interrupt nesting behaviour configuration.
#define configKERNEL_INTERRUPT_PRIORITY \
(configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8U - __NVIC_PRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8U - __NVIC_PRIO_BITS))

// Define to trap errors during development.
#define configASSERT(x) assert_param(x)

// Optional functions - most linkers will remove unused functions anyway.
#define INCLUDE_vTaskPrioritySet                0
#define INCLUDE_uxTaskPriorityGet               0
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    1 // for portMAX_DELAY
#define INCLUDE_xResumeFromISR                  0
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          0
#define INCLUDE_xTaskGetCurrentTaskHandle       0
#if defined(DEBUG)
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_uxTaskGetStackHighWaterMark2    0
#endif
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xEventGroupSetBitFromISR        0
#define INCLUDE_xTimerPendFunctionCall          0
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              0

// Definitions that map the FreeRTOS port interrupt handlers to their CMSIS standard names.
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
