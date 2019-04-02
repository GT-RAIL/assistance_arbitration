# A message type to provide an intervention trace

# A header indicating the time of the trace update
time stamp

# The type of the event
uint8 INTERVENTION_START_EVENT=1
uint8 HYPOTHESIS_EVENT=2
uint8 ACTION_EVENT=3
uint8 type

# The different types of events sometimes have associated metadata
InterventionHypothesisMetadata hypothesis_metadata
InterventionActionMetadata action_metadata
