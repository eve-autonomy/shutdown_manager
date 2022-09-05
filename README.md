# Shutdown manager

## Overview

This node accepts button operation input and controls shutdown. It also outputs to display the button operation status.

## Input and Output

- Input
  - from [button_output_selector](https://github.com/eve-autonomy/button_output_selector/)
    - `/shutdown_button` \[[autoware_state_machine_msgs/msg/VehicleButton][ReservationButton]\]:<br>Shutdown button input. (this topic is remapped from `input/shutdown_button`.)

- Output
  - to [delivery_reservation_lamp_manager](https://github.com/eve-autonomy/delivery_reservation_lamp_manager/)
    - `/shutdown_manager/state` \[[shutdown_manager_msgs/msg/StateShutdown][StateShutdown]\]:<br>Output to shutdown state. (this topic is remapped from `output/state`.)

[StateShutdown]: https://github.com/eve-autonomy/shutdown_manager_msgs/blob/main/msg/StateShutdown.msg
[ReservationButton]: https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/VehicleButton.msg

## Node Graph

![node_graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/shutdown_manager/main/docs/node_graph.pu)

## Launch Parameter description

|ID|Name|Description|
|:---|:---|:----------|
|T0|time_required_to_release_button|Button hold down time[sec] required before shutdown is accepted.|
|T1|timeout_period_before_shutdown_aborts|Time[sec] given between accepting and executing shutdown (timeout after this time).|

## Shutdown state transition

![state_transition](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/shutdown_manager/main/docs/state_transition.pu)
