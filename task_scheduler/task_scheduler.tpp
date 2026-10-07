// Note that the template header is required on all non-inline functions in .tpp files

#pragma once

template <size_t MaxTasks, size_t MaxDepsPerTask>
TaskScheduler<MaxTasks, MaxDepsPerTask>::TaskScheduler() {}

template <size_t MaxTasks, size_t MaxDepsPerTask>
std::optional<size_t> TaskScheduler<MaxTasks, MaxDepsPerTask>::AddTask(TaskCallback cb, size_t requiredDeps) {
    if (requiredDeps > MaxDepsPerTask) return std::nullopt;

    for (size_t i = 0; i < MaxTasks; i++) {
        if (!_tasks[i].callback) {
            _tasks[i].callback = cb;
            _tasks[i].requiredDependencies = requiredDeps;
            _tasks[i].active = false;
            _tasks[i].next = MaxTasks;
            
            for (size_t d = 0; d < MaxDepsPerTask; ++d) {
                _tasks[i].dependencies[d] = false;
            }
            return i;
        }
    }
    return std::nullopt; 
}

template <size_t MaxTasks, size_t MaxDepsPerTask>
void TaskScheduler<MaxTasks, MaxDepsPerTask>::ScheduleTask(size_t id, uint32_t targetTime) {
    if (id >= MaxTasks || !_tasks[id].callback) return;

    // Disable interrupts temporarily to prevent superloop modifying task at the same time
    SCHEDULER_CRITICAL_ENTER();

    RemoveFromList(id);

    _tasks[id].scheduledTime = targetTime;
    _tasks[id].active = true;
    _tasks[id].next = MaxTasks;

    // If list is empty OR targetTime is chronologically BEFORE the head's time,
    // move the new task to the front
    if (_head == MaxTasks || TimeIsBefore(targetTime, _tasks[_head].scheduledTime)) {
        _tasks[id].next = _head;
        _head = id;
    } else {
        size_t curr = _head;
        
        // Iterate until we find a task scheduled LATER than our target time
        while (_tasks[curr].next != MaxTasks && 
               !TimeIsBefore(targetTime, _tasks[_tasks[curr].next].scheduledTime)) {
            curr = _tasks[curr].next;
        }
        _tasks[id].next = _tasks[curr].next;
        _tasks[curr].next = id;
    }

    // Re-enable interrupts
    SCHEDULER_CRITICAL_EXIT();
}

template <size_t MaxTasks, size_t MaxDepsPerTask>
void TaskScheduler<MaxTasks, MaxDepsPerTask>::SetDependency(size_t id, size_t depIndex, bool state) {
    if (id < MaxTasks && depIndex < _tasks[id].requiredDependencies) {
        _tasks[id].dependencies[depIndex] = state;
    }
}

template <size_t MaxTasks, size_t MaxDepsPerTask>
void TaskScheduler<MaxTasks, MaxDepsPerTask>::Run(uint32_t currentTime) {
    size_t curr = _head;
    size_t prev = MaxTasks;

    // Iterates over the sorted linked list of tasks until finding the first task
    // whose time hasn't passed yet
    // Executes all tasks for which the time and dependency requirements have been met
    while (curr != MaxTasks) {
        Task& t = _tasks[curr];

        // Bail out instantly if the current time has not reached the scheduled time
        // This prevents us from checking any future tasks after this one
        if (!TimeIsReady(currentTime, t.scheduledTime)) {
            break;
        }

        // Only check the active dependencies for this specific task
        bool depsMet = true;
        for (size_t i = 0; i < t.requiredDependencies; ++i) {
            if (!t.dependencies[i]) {
                depsMet = false;
                break;
            }
        }

        if (depsMet) {
            // Disable interrupts temporarily to prevent superloop modifying task at the same time
            SCHEDULER_CRITICAL_ENTER();
            size_t nextNode = t.next;
            
            // Remove task from the linked list
            if (prev == MaxTasks) {
                _head = nextNode;
            } else {
                _tasks[prev].next = nextNode;
            }

            // Re-enable interrupts
            SCHEDULER_CRITICAL_EXIT();

            // Clear state to prevent race conditions during self-rescheduling
            t.active = false;
            t.next = MaxTasks;
            for (size_t i = 0; i < t.requiredDependencies; ++i) {
                t.dependencies[i] = false;
            }

            // Execute the task
            t.callback();

            // If the task was executed, we don't update prev since that task is not active now
            curr = nextNode;
        } else {
            // Time is met but dependencies aren't; keep iterating
            prev = curr;
            curr = t.next;
        }
    }
}

template <size_t MaxTasks, size_t MaxDepsPerTask>
inline bool TaskScheduler<MaxTasks, MaxDepsPerTask>::TimeIsBefore(uint32_t t1, uint32_t t2) {
    return (int32_t)(t1 - t2) < 0;
}

template <size_t MaxTasks, size_t MaxDepsPerTask>
inline bool TaskScheduler<MaxTasks, MaxDepsPerTask>::TimeIsReady(uint32_t current, uint32_t scheduled) {
    return (int32_t)(current - scheduled) >= 0;
}

template <size_t MaxTasks, size_t MaxDepsPerTask>
void TaskScheduler<MaxTasks, MaxDepsPerTask>::RemoveFromList(size_t id) {
    if (_head == MaxTasks) return;
    if (_head == id) {
        _head = _tasks[id].next;
        return;
    }
    size_t curr = _head;
    while (_tasks[curr].next != MaxTasks) {
        if (_tasks[curr].next == id) {
            _tasks[curr].next = _tasks[id].next;
            return;
        }
        curr = _tasks[curr].next;
    }
}