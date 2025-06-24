# Agent Task Instructions

This directory contains instructions for automated agents to perform specific tasks. Each file is named `<task_name>.md` and provides detailed steps that an agent should follow.

## How to Use

When an agent is directed to perform a task:

1. Identify the appropriate instruction file by its task name, e.g., `basilisk-c-shim.md`.
2. Open and parse the file to determine:
   - **Purpose** of the task
   - **Prerequisites** (inputs, environment)
   - **Outputs** (expected results)
   - **Step-by-step procedure**
3. Execute the documented steps.
4. Report back any errors, ambiguities, or updates needed.

## Task Index

- **basilisk-c-shim.md**: Create C shim layers for Basilisk C++ algorithms to interface with an Ada system. Converts complex C++ types into plain C data structures (PODs), defines opaque pointers, constructors, destructors, and methods with C linkage.

*(Add additional tasks below as separate `.md` files are created.)*

## File Naming and Version Control

- Instruction files must be named exactly `<task_name>.md`.
- Place all instruction files at the top level of this directory.
- When adding or updating a task, commit with clear messages, e.g.,:
  - `docs(agent): add data-cleanup task instructions`
  - `docs(agent): update basilisk-c-shim procedure`

## Contributing New Tasks

1. Create a new file named `<task_name>.md` in this directory.
2. Include sections:
   - **Task Name**
   - **Purpose**
   - **Prerequisites**
   - **Inputs/Outputs**
   - **Procedure**
3. Test with the intended agent implementation.
4. Submit a pull request for review.

## Maintenance

- Periodically review and update instruction files for accuracy.
- Deprecate obsolete tasks by marking the file with `DEPRECATED` in the title and removing from the index.

_Last updated: 2025-06-24_

