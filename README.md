# ROS 2 Action Server

This repository demonstrates **ROS 2 Action Servers** with example code, explanations, and projects using them.  

Actions in ROS 2 are used when you want to **send a goal to a node**, get **continuous feedback** about the goal's progress, and finally receive the **result** when the goal completes.  

Think of it like giving someone a task, **checking in as they progress**, and then finally confirming the task is completed successfully. ✅  

---

## Concept of ROS 2 Actions

A ROS 2 Action consists of three main components:

1. **Action Server** – The node that executes the goal.  
2. **Action Client** – The node that sends the goal and monitors progress.  
3. **Action Definition** – The `.action` file defining **Goal**, **Result**, and **Feedback**.

> The action topic name must be **the same** in both the client and the server.  

---

### Anatomy of a `.action` file

Before creating an action server, we need to define a `.action` file.  
This file contains three parts:

1. **Goal** – What the client wants the server to do.  
2. **Result** – What the server returns after completing the goal.  
3. **Feedback** – Continuous updates about the goal's progress.

![Robot Action](https://docs.ros.org/en/jazzy/_images/Action-SingleActionClient.gif)

Example: `DiffDrive.action`

```plaintext
# Goal
int32 distance  # distance to move in meters
---
# Result
bool success    # whether the goal was successful
---
# Feedback
int32 current_distance  # current distance covered
