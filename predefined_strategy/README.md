# Predfined Strategy

The strategies here are predefined by the developers based on expected errors. If the current situation does not match an expected error, then the strategy returns a `RESUME_NONE`.

## Notes

API for `recovery_strategies`:

```python
class RecoveryStrategies(object):

    def __init__(self, tasks_config):
        """
        Args:
            tasks_config (dict)
        """
        pass

    def init(self):
        pass

    def get_strategy(self, assistance_goal):
        """
        Args:
            assistance_goal (assistance_msgs/RequestAssistanceGoal)

        Returns:
            (tuple) :
                - execute_goal (assistance_msgs/ExecuteGoal)
                - resume_hint (int) : assistance_msgs/RequestAssistanceResult
                - resume_context (dict)
        """
        return (None, None, None)
```
