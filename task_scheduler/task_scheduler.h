#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <optional>

// Mock the interrupt enable/disable for testing purposes
#ifdef __arm__
    #include "stm32f1xx_hal.h"
    #define SCHEDULER_CRITICAL_ENTER() uint32_t primask = __get_PRIMASK(); __disable_irq()
    #define SCHEDULER_CRITICAL_EXIT()  __set_PRIMASK(primask)
#else
    #include <mutex>
    extern std::mutex test_mutex;
    #define SCHEDULER_CRITICAL_ENTER() test_mutex.lock()
    #define SCHEDULER_CRITICAL_EXIT()  test_mutex.unlock()
#endif

// A task defines its callback as a generic function pointer
typedef void (*TaskCallback)();

template <size_t MaxTasks, size_t MaxDepsPerTask>
class TaskScheduler {
public:
    struct Task {
        TaskCallback callback = nullptr;
        // Tasks schedule themselves for certain timestamps
        uint32_t scheduledTime = 0;
        
        std::array<bool, MaxDepsPerTask> dependencies = {false};
        // The number of required dependencies to check for this task
        // Must be <= `MaxDepsPerTask`; dependencies beyond this number are not checked
        size_t requiredDependencies = 0; 
        
        // Whether the task is scheduled to execute again
        bool active = false;
        // Index of the next task in the linked list
        size_t next = MaxTasks; 
    };

    TaskScheduler();

    /**
     * @brief Allocates a task to the next available open slot.
     * @param cb The function to execute.
     * @param requiredDeps How many dependencies this specific task needs (up to `MaxDepsPerTask`).
     * @retval The task id, if created successfully, or std::nullopt otherwise.
     */
    std::optional<size_t> AddTask(TaskCallback cb, size_t requiredDeps = 0);

    /**
     * @brief Safely sorts a task by execution time into the execution list.
     * @param id The id of the task to schedule
     * @param targetTime When the task should attempt to be executed; this is the time
     * that's used to insert the task into the list in its sorted position
     */
    void ScheduleTask(size_t id, uint32_t targetTime);

    /**
     * @brief Sets the state of a single task dependency.
     * @param id The task id, assigned on creation.
     * @param depIndex The index of the specific dependency in the task's array.
     * @param state Whether the dependency has been met or not.
     */
    void SetDependency(size_t id, size_t depIndex, bool state);

    /**
     * @brief Runs all ready tasks in a single cycle, evaluating timers and dependencies, 
     * @param currentTime The time to compare the scheduled time of the tasks against.
     */
    void Run(uint32_t currentTime);

private:
    std::array<Task, MaxTasks> _tasks;
    volatile size_t _head = MaxTasks;

    /**
     * @brief Compares two timestamps, handling hardware timer rollovers.
     * Evaluates true if t1 is chronologically before t2.
     */
    static inline bool TimeIsBefore(uint32_t t1, uint32_t t2);

    /**
     * @brief Evaluates true if the current time has reached or passed the scheduled time.
     */
    static inline bool TimeIsReady(uint32_t current, uint32_t scheduled);

    /**
     * @brief Helper function to remove a task from the linked list of scheduled tasks.
     */
    void RemoveFromList(size_t id);
};

// Include the implementation file directly so the compiler can access the template logic
// Template classes can't use standard .cpp files
#include "task_scheduler.inl"