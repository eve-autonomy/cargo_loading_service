# cargo_loading_service

## Overview
Cargo-Loading Service module gives control instructions to infrastructure.<br>
There are three types of control instructions: process start, end, and error.

## Input and Output
- input
  - from [in_parking_task_manager](https://github.com/tier4/in_parking_task_manager)
    - `/in_parking/state` \[[in_parking_msgs/msg/InParkingStatus](https://github.com/tier4/in_parking_msgs/blob/main/msg/InParkingStatus.msg)\]:<br>State at vehicle stop points.
  - from [v2i_interface](https://github.com/eve-autonomy/cargo_loading_service)
    - `/v2i/infrastructer_states` \[[v2i_interface_msgs/msg/InfrastructureStateArray.msg](https://github.com/eve-autonomy/v2i_interface_msgs/blob/main/msg/InfrastructureState.msg)\]:<br>Loading result notification.
- input service
  - from [in_parking_task_manager](https://github.com/tier4/in_parking_task_manager)
    - `/in_parking/task`\[[in_parking_msgs/srv/ExecuteInParkingTask.srv](https://github.com/tier4/in_parking_msgs/blob/main/srv/ExecuteInParkingTask.srv)\]:<br>This is a service that is called when it is necessary to  cargo loading.
- output
  - to [v2i_interface](https://github.com/eve-autonomy/v2i_interface)
    - `/cargo_loding/infurastructre_commands` \[[v2i_interface_msgs/msg/InfrastructureCommandArray](https://github.com/eve-autonomy/v2i_interface_msgs/blob/main/msg/InfrastructureCommandArray.msg)\]:<br>Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/cargo_loading_service/main/docs/node_graph.pu)

## Launch arguments
none
