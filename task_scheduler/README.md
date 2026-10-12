# Firmware Task Scheduler

## Overview

This is a minimal, single-threaded cooperative task scheduler designed for STM32 embedded firmware. It runs callback functions at target timestamps once all configured dependencies have been met.

It is **timer-agnostic**: it operates on abstract 32-bit unsigned timestamps (`uint32_t`), allowing it to use `HAL_GetTick()` for millisecond intervals or arbitrary resolution hardware timer registers (such as STM32 `TIM4` or `TIM5` counter registers).

## Usage

### Task Initialization

The scheduler is implemented as a template class parameterized by capacity limits, avoiding dynamic memory allocation:

```cpp
template <size_t MaxTasks, size_t MaxDepsPerTask>
class TaskScheduler;
```

- `MaxTasks`: The maximum number of tasks that can be allocated concurrently in the internal task table.
- `MaxDepsPerTask`: The maximum number of prerequisite dependency flags tracked per task.

Tasks are registered using the `AddTask()` method:

```cppf
std::optional<size_t> AddTask(TaskCallback cb, size_t requiredDeps = 0);
```

- `cb`: A function pointer (`void (*)()`) to execute when the task is triggered.
- `requiredDeps`: The number of boolean dependencies this task requires before running (must be `<= MaxDepsPerTask`). Defaults to `0`.

`AddTask()` allocates the next available open slot in the static task table and returns the assigned `size_t` task ID. If `requiredDeps` exceeds `MaxDepsPerTask` or if the task table is full, it returns `std::nullopt`. Tasks are registered in an inactive state and must be explicitly scheduled before they will execute.

### Scheduling Tasks

Tasks are queued for execution using the `ScheduleTask()` method:

```cpp
void ScheduleTask(size_t id, uint32_t targetTime);
```

- `id`: The task ID returned by `AddTask()`.
- `targetTime`: The timestamp at which the task should attempt execution, expressed in ticks of the application's chosen timebase (e.g. `TIM5->CNT` or `HAL_GetTick()`).

`ScheduleTask()` inserts the task into a singly linked list sorted chronologically by `targetTime`. If the task was already active, it is unlinked and re-inserted at the new target time.

Scheduling is interrupt-safe: `ScheduleTask()` enters a critical section (`SCHEDULER_CRITICAL_ENTER()`), disabling interrupts on ARM platforms (`__disable_irq()`) to prevent race conditions when tasks are scheduled from interrupt service routines (ISRs) or preempted by the superloop.

Note: **the scheduler does not automatically reschedule tests**. To run a task periodically, the task callback should reschedule itself by calling `ScheduleTask(id, lastScheduledTime + interval)`.

### Task Dependencies

Tasks can declare up to `MaxDepsPerTask` boolean prerequisites that must all evaluate to `true` before the callback will execute. Dependencies are updated using `SetDependency()`:

```cpp
void SetDependency(size_t id, size_t depIndex, bool state);
```

- `id`: The target task ID.
- `depIndex`: The index of the dependency (`0` to `requiredDependencies - 1`).
- `state`: Boolean status flag indicating whether the prerequisite has been met.

Dependencies are also interrupt-safe (see [Scheduling Tasks](#scheduling-tasks)).

**Dependencies are automatically reset**: When a task's callback runs, the scheduler resets all of that task's active dependencies back to `false`. Subsequent executions require dependencies to be explicitly re-asserted, preventing stale event flags from inadvertently triggering repeat executions.

If a task reaches its scheduled time but one or more dependencies remain unmet, **the task remains in the active list** and will be re-evaluated on subsequent scheduler cycles (so it will run as soon as the depedencies are fulfilled).

### Main Scheduler Loop

The scheduler is driven by calling `Run()` repeatedly within the firmware superloop:

```cpp
void Run(uint32_t currentTime);
```

- `currentTime`: The current hardware timestamp sampled from the application's timebase (e.g. `TIM5->CNT` or `HAL_GetTick()`).

During each pass, `Run()` traverses the chronologically sorted task list:
1. Exit early: If the current time has not reached the head task's scheduled timestamp, `Run()` terminates without checking later tasks.
2. Check depedencies: For each task whose target time has arrived, `Run()` verifies that all required dependencies are satisfied.
3. Execute task: If all dependencies are satisfied, the task is removed from the task list, its active and dependency states are reset, and its callback is executed. This is repeated for all tasks that are ready to execute.
4. Deferred tasks: If dependencies are not yet met, the task remains in the list and `Run()` advances to evaluate subsequent tasks whose scheduled time may also have passed.

## Current Limitations

### Lack of Task Priority

Tasks are scheduled and evaluated strictly based on chronological order (`targetTime`). If multiple tasks expire at the same time, they are dispatched in the order they were inserted (FIFO) based on task ID.

There is currently no support for task priorities or preemption levels. Future revisions could introduce priority queues or tie-breaking priority fields.

### Lack of Multithreading

The scheduler operates in a single-threaded, cooperative execution model:
- Callbacks run synchronously on the superloop's execution context.
- **A long-running or blocking task callback will stall the scheduler and starve all other pending tasks**.
- There is no thread preemption or hardware context switching. Callbacks must remain lightweight and non-blocking, delegating long delays to timer scheduling or asynchronous interrupts.

### Lack of Callback Context / User Arguments

Task callbacks are currently defined as bare function pointers without parameters:

```cpp
typedef void (*TaskCallback)();
```

Because callbacks cannot receive user-supplied data (such as a `void*` context pointer, an object reference, or even the task's own ID), tasks that reschedule themselves or interact with other tasks must access state and IDs via static or global variables.

This makes integrating the scheduler into applications more cumbersome (requiring static wrappers and extra boilerplate). Future revisions could support context arguments (e.g. `void (*)(void* user_data)` or passing the `size_t taskId` directly into the callback).

For now, it is recommended to encapsulate tasks, callbacks, and necessary state in a **controller class separate from the rest of the application** to ensure the global namespace doesn't get too polluted.

## Basic Usage Example

The example below demonstrates scheduling tasks using a hardware timer register (such as 32-bit timer `TIM5` at a configurable resolution, but this is just a made-up example) and signaling a dependency from an external GPIO interrupt: Note that you may want to contain these global variables within a user-defined class instead.

```cpp
#include <optional>
#include "stm32f1xx_hal.h"
#include "task_scheduler/task_scheduler.h"

// 1. Declare scheduler instance (e.g. up to 8 tasks, 2 dependencies per task)
TaskScheduler<8, 2> scheduler;

// Task IDs (stored globally so callbacks and ISRs can reference them)
size_t sensorTaskId = 0;
size_t telemetryTaskId = 0;

// Helper to sample the hardware timebase (e.g. HAL_GetTick, whatever other timer)
inline uint32_t GetCurrentTime() {
    return TIM5->CNT;
}

// 2. Define task callbacks
void SensorTask() {
    // Initiate non-blocking sensor acquisition (e.g., start DMA or async read)...

    // Reschedule self to poll again in 10,000 timer ticks (e.g. 10 ms at 1 MHz TIM5)
    scheduler.ScheduleTask(sensorTaskId, GetCurrentTime() + 10000);
}

void TelemetryTask() {
    // Transmit telemetry packet (only runs when dependency 0 is satisfied)
    // Dependencies are automatically reset to false by the scheduler after running

    // Reschedule self for the next transmission window (e.g. 100,000 timer ticks)
    scheduler.ScheduleTask(telemetryTaskId, GetCurrentTime() + 100000);
}

// 3. Hardware interrupt handler asserting a task dependency
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        // Sensor data-ready interrupt: satisfy dependency 0 for telemetry task
        // SetDependency() is interrupt-safe and can be called directly from ISRs
        scheduler.SetDependency(telemetryTaskId, 0, true);
    }
}

int main() {
    HAL_Init();
    SystemClock_Config();

    // Init rest of hardware

    // 4. Register tasks with AddTask
    std::optional<size_t> sensorId = scheduler.AddTask(SensorTask, 0);          // No dependencies
    std::optional<size_t> telemetryId = scheduler.AddTask(TelemetryTask, 1);    // 1 required dependency

    if (!sensorId.has_value() || !telemetryId.has_value()) {
        // Handle task registration failure (e.g. task table capacity reached)
        while (1);
    }

    sensorTaskId = *sensorId;
    telemetryTaskId = *telemetryId;

    // 5. Schedule initial execution timestamps
    uint32_t now = GetCurrentTime();
    scheduler.ScheduleTask(sensorTaskId, now + 10000);
    scheduler.ScheduleTask(telemetryTaskId, now + 100000);

    // 6. Main superloop: continuously process ready tasks
    while (1) {
        scheduler.Run(GetCurrentTime());
    }
}
```