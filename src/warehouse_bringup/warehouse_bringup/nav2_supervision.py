"""
Passive Nav2 supervision helpers used by the mission manager.

Nothing here commands the robot. These classes only interpret messages the
mission manager already receives:

* ActionActivityTracker: active goals per Nav2 action server, from each
  server's ``<action>/_action/status`` topic. A failed NavigateToPose does not
  mean Nav2 is idle: Trial 08 logged spin/backup/follow_path goals still
  running for about a minute after the outer goal aborted.
* MotionTracker: whether odometry shows the robot as stationary, and for how
  long, measured in both simulation time and wall time.
"""

import math

from action_msgs.msg import GoalStatus

ACTIVE_GOAL_STATES = frozenset((
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
))


class ActionActivityTracker:
    """Track which Nav2 action servers report goals that are still active."""

    def __init__(self):
        self._active = {}
        self._busy_during_attempt = set()

    def begin_attempt(self):
        self._busy_during_attempt.clear()

    def saw_activity(self, action_name):
        return action_name in self._busy_during_attempt

    def update(self, action_name, status_list):
        """Record the latest GoalStatusArray.status_list for an action server."""
        self._active[action_name] = sum(
            1 for status in status_list if status.status in ACTIVE_GOAL_STATES
        )
        if self._active[action_name]:
            self._busy_during_attempt.add(action_name)

    def active_actions(self):
        """Return {action_name: active_goal_count} for servers with active goals."""
        return {name: n for name, n in self._active.items() if n > 0}

    def idle(self):
        return not self.active_actions()


class MotionTracker:
    """Decide from odometry whether the robot is stationary."""

    def __init__(self, linear_tol=0.01, angular_tol=0.02, max_gap_sec=2.0):
        self.linear_tol = linear_tol
        self.angular_tol = angular_tol
        self.max_gap_ns = max_gap_sec * 1e9
        self.last_msg_sim_ns = None
        self.last_msg_wall_ns = None
        self.reset_hold()

    def reset_hold(self):
        self.stationary_since_sim_ns = None
        self.stationary_since_wall_ns = None

    def update(self, linear_x, linear_y, angular_z, sim_ns, wall_ns):
        if (self.last_msg_wall_ns is None or
                wall_ns - self.last_msg_wall_ns > self.max_gap_ns or
                not 0 < sim_ns - self.last_msg_sim_ns <= self.max_gap_ns):
            self.reset_hold()
        self.last_msg_sim_ns = sim_ns
        self.last_msg_wall_ns = wall_ns
        moving = (
            not all(math.isfinite(v) for v in (linear_x, linear_y, angular_z)) or
            abs(linear_x) > self.linear_tol or
            abs(linear_y) > self.linear_tol or
            abs(angular_z) > self.angular_tol
        )
        if moving:
            self.reset_hold()
        elif self.stationary_since_wall_ns is None:
            self.stationary_since_sim_ns = sim_ns
            self.stationary_since_wall_ns = wall_ns

    def fresh(self, wall_ns, max_age_sec):
        """Return True if odometry arrived within max_age_sec of wall time."""
        return (self.last_msg_wall_ns is not None and
                wall_ns - self.last_msg_wall_ns <= max_age_sec * 1e9)

    def stationary_for_sim(self, sim_ns, hold_sec):
        return (self.stationary_since_sim_ns is not None and
                min(sim_ns, self.last_msg_sim_ns) - self.stationary_since_sim_ns >= hold_sec * 1e9)

    def stationary_for_wall(self, wall_ns, hold_sec):
        if self.stationary_since_wall_ns is None:
            return False
        end_ns = min(wall_ns, self.last_msg_wall_ns)
        return end_ns - self.stationary_since_wall_ns >= hold_sec * 1e9
