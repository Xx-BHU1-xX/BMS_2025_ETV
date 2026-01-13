## Firmware overview

The BMS firmware runs on the ESP32 using FreeRTOS and is structured as a set of independent tasks, each responsible for one clear function (cell monitoring, SOC/DCIR estimation, current sensing, balancing, fault handling, comms, etc). Most tasks are implemented as simple state machines and communicate with each other using RTOS primitives. The overall focus is on robustness, predictable behavior, and making sure safety-critical logic (especially SDC control) cannot fail.

---

## General firmware practices

### RTOS usage
- Standard FreeRTOS practices are followed:
  - one task per logical function
  - clear ownership of peripherals
  - explicit synchronization between tasks

---

### Data sharing between tasks
- Data between tasks is shared using **queues**.
- In most cases, only the **latest value** matters and older values are irrelevant.
- Hence, almost all queues are created with **length = 1**.
- Writes use `xQueueOverwrite()` to always push the latest value.
- Reads use `xQueuePeek()` so values can be read **non-destructively**.
- This avoids issues like conflicting writes and read/write races.
- Stale data detection is additonaly implemented in fault detection and handling.

---

### Resource management
- Shared peripherals (I2C, SPI, CAN, UART, etc.) are protected using **mutexes**.
- If a task needs a peripheral, it:
  1. takes the mutex  
  2. uses the peripheral  
  3. releases the mutex
- Once a task has taken a mutex, no other task can access that peripheral, even if the original task gets preempted.
- This ensures the reason the task needed the peripheral in the first place doesn’t get corrupted by another task.

---

### Interrupt handling
- ISRs themselves are not part of the RTOS.
- Interrupts are handled using **short ISRs** that simply notify a waiting task.
- The actual processing is always done in task context.

**Important**
- The task must already be waiting for the interrupt **before** the interrupt fires.
- In practice, this means:
  - perform the action
  - wait for the interrupt **in the same state**
- The wait is intentionally not moved into the next state, because any delay between states could cause the interrupt to arrive when the task isn’t waiting, adding unnecessary latency or missing the event altogether.


For task descriptions, refer 
[Full task descriptions (PDF)](docs/BMS_25.pdf)

