#pragma once

/**
 * @brief Scheduler class
 *
 * @defgroup scheduler
*/

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <etl/vector.h>
#include <etl/delegate.h>

#include "logging/logging.hpp"

enum class TaskType {
  PERIODIC,
  CONTINUOUS
};

template <typename TDelegate, size_t StackSize, TaskType task_type = TaskType::PERIODIC>
struct StaticTaskHolder {
  const char* name;
  uint32_t frequencyHz;
  uint32_t priority;
  TDelegate taskFunction;
  StackType_t stack[StackSize];
  StaticTask_t taskData;
  TaskHandle_t handle = nullptr;
};

template <typename TDelegate, TaskType task_type = TaskType::PERIODIC>
struct DynamicTaskHolder {
  const char* name;
  uint32_t frequencyHz;
  uint32_t priority;
  TDelegate taskFunction;
  uint32_t stackSize;
  SemaphoreHandle_t taskReadySemaphore;
};

class Scheduler {
public:
  Scheduler();

  void Init();
  // Notify a specific static task from an ISR using its task handle
  template<typename TDelegate, size_t StackSize, TaskType task_type>
  void NotifyFromISR(const StaticTaskHolder<TDelegate, StackSize, task_type>& task)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (task.handle != nullptr) {
      xTaskNotifyFromISR(task.handle, 0, eNoAction, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }

  void NotifyGiveFromISR(TaskHandle_t handle)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (handle != nullptr) {
      vTaskNotifyGiveFromISR(handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }

  template<typename TDelegate, size_t StackSize, TaskType task_type>
  void NotifyGiveFromISR(const StaticTaskHolder<TDelegate, StackSize, task_type>& task)
  {
    NotifyGiveFromISR(task.handle);
  }

  /**
   * @brief Create a FreeRTOS static task. Thanks to etl::delegate we can easily create static tasks that point to a class member 
   * function with compile-time construction.
   * 
   * @tparam TDelegate The delegate type
   * @tparam StackSize The stack size of the task
   * @param task The task holder
   * 
   * @note The StaticTaskHolder object must be static or global to ensure the stack memory is not deallocated during task execution.
   * 
   * @ingroup scheduler
   */
  template<typename TDelegate, size_t StackSize, TaskType task_type>
  void CreateStaticTask(StaticTaskHolder<TDelegate, StackSize, task_type>& task)
  {
    TaskHandle_t handle = xTaskCreateStatic(
      [] (void* pvParameters) {
        StaticTaskHolder<TDelegate, StackSize, task_type>* task = static_cast<StaticTaskHolder<TDelegate, StackSize, task_type>*>(pvParameters); 
        if constexpr (task_type == TaskType::PERIODIC) {
          // Periodic task launcher
          TickType_t xLastWakeTime = xTaskGetTickCount();
          const TickType_t xFrequency = pdMS_TO_TICKS(1000 / task->frequencyHz);
          for (;;) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            task->taskFunction(); // Call the task function
          }
        } else {
          // Continuous task launcher
          for (;;) {
            task->taskFunction(); // Call the task function
          }
        }
      },
      task.name,
      StackSize,
      &task,
      task.priority,
      task.stack,
      &task.taskData
    );
    task.handle = handle;
  }

  template<typename TDelegate, size_t StackSize, TaskType task_type>
  void CreateStaticPinnedTask(StaticTaskHolder<TDelegate, StackSize, task_type>& task,
                              BaseType_t coreId)
  {
    TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
      [] (void* pvParameters) {
        StaticTaskHolder<TDelegate, StackSize, task_type>* task = static_cast<StaticTaskHolder<TDelegate, StackSize, task_type>*>(pvParameters);
        if constexpr (task_type == TaskType::PERIODIC) {
          TickType_t xLastWakeTime = xTaskGetTickCount();
          const TickType_t xFrequency = pdMS_TO_TICKS(1000 / task->frequencyHz);
          for (;;) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            task->taskFunction();
          }
        } else {
          for (;;) {
            task->taskFunction();
          }
        }
      },
      task.name,
      StackSize,
      &task,
      task.priority,
      task.stack,
      &task.taskData,
      coreId
    );
    task.handle = handle;
  }

  /**
   * @brief Create a FreeRTOS dynamic task. Thanks to etl::delegate we can easly create dynamic tasks that point to a class member 
   * function with compile-time construction.
   * 
   * @tparam TDelegate The delegate type
   * @param task The task holder
   * 
   * @note Unlike CreateStaticTask, this function will allocate memory on the heap for the task stack and the DynamicTaskHolder objects lifetime
   * only needs to be until the task is created.
   * 
   * 
   * @note THe calling thread cannot have higher priority than the created task. This is due to the fact that the calling thread will be blocked
   * 
   * @ingroup scheduler
  */
  template<typename TDelegate, TaskType task_type>
  TaskHandle_t CreateDynamicTask(DynamicTaskHolder<TDelegate, task_type>& task)
  {
    TaskHandle_t handle = nullptr;
    task.taskReadySemaphore = xSemaphoreCreateBinary();
    BaseType_t err = xTaskCreate(
      [] (void* pvParameters) {
        DynamicTaskHolder<TDelegate, task_type>* task = static_cast<DynamicTaskHolder<TDelegate, task_type>*>(pvParameters);
        
        if constexpr (task_type == TaskType::PERIODIC) {
          // Periodic task launcher
          TickType_t xLastWakeTime = xTaskGetTickCount();
          const TickType_t xFrequency = pdMS_TO_TICKS(1000 / task->frequencyHz);

          TDelegate taskFunction = task->taskFunction;    // To make sure the delegate is copied to the stack so DynamicTaskHolder can be deleted
          xSemaphoreGive(task->taskReadySemaphore);       // Signal that the task is ready

          for (;;) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            taskFunction(); // Call the task function
          }
        } else {
          // Continuous task launcher
          TDelegate taskFunction = task->taskFunction;    // To make sure the delegate is copied to the stack so DynamicTaskHolder can be deleted
          xSemaphoreGive(task->taskReadySemaphore);       // Signal that the task is ready

          for (;;) {
            taskFunction(); // Call the task function
          }
        }
      },
      task.name,
      task.stackSize,
      &task,
      task.priority,
      &handle
    );

    if (err != pdPASS) {
      LOG_ERROR("Failed to create task: %s", task.name);
    } else {
      xSemaphoreTake(task.taskReadySemaphore, portMAX_DELAY);
    }

    return handle;
  }

public:
  static Scheduler scheduler;

private:
  static constexpr uint32_t MAX_TASKS = 10;
};
