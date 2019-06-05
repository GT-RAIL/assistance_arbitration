# Assistance Arbitrator

Given an incoming assistance goal, the arbitrator uses a method selected through config files to decide how to resolve the request. When it picks a method of resolution, called a **strategy**, it then forwards the request on to the appropriate strategy.

The strategies are:

1. [Predefined Strategy](../predefined_strategy/): The developer has pre-specified different strategies for different error conditions.
1. [Local Strategy](../local_strategy/): The robot looks for a local human, approaches them, and tries to solicit help from this human.
1. [Remote Strategy](../remote_strategy/): The robot sends a request to a remote interface, which is then presented to a user on that interface.
