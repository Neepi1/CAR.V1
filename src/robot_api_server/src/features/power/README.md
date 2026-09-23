# Power

Positive current alone now requires existing confirmed dock occupancy; ordinary
navigation, teleop and predock cannot create contact from it. Raw current is
retained. The context callback runs without the power mutex and never calls the
BMS-derived occupancy evaluation. Deployed 2026-09-18; physical acceptance pending; see
[scope and tests](../../../../../docs/bms_current_scope.md).

Owns the complete process-resident Ranger `BatteryState` input boundary:

- the single ROS subscription and its QoS;
- normalized SOC and charging-contact interpretation;
- contact/no-contact stability timing and freshness expiry;
- the immutable BMS snapshot consumed by status, teleop, navigation admission,
  docking occupancy, and controlled undock;
- ordered callbacks after snapshot commit: docking-latch evidence first, then
  the teleop charging-contact update.

`power_configuration_module` is the sole declaration owner for the eight BMS
evidence parameters: topic/freshness, current and voltage thresholds, full-SOC
interpretation, and contact-stability duration. The deployed
`teleop_charging_current_min_a` parameter name is retained for compatibility,
but the value is shared BMS evidence and is projected from this module into
docking. `teleop_stop_on_charging` remains a teleop-owned policy and is not
absorbed here.

The composition root supplies downstream callbacks and projects the immutable
configuration into neighboring modules only. It does not retain parallel raw
BMS fields, parameter scalars, or subscription state. This module does not
command charging, undocking, docking, estop, or chassis motion, and the
extraction adds no new admission gate or BMS decision rule.
