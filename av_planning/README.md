# av_planning

The `av_planning` package is responsible for planning and executing the active vision tasks for the Igus ReBeL robot. This package includes source code, launch files, and configuration files required for the predefined and next-best-view (NBV) planning pipelines.


## Key Components

- **active_vision_pipeline.hpp**: This header file defines the `ActiveVisionPipeline` class, which serves as the base class for the planning pipelines.
- **active_vision_predefined_planning_pipeline.hpp**: This header file defines the `ActiveVisionPredefinedPlanningPipeline` class, which handles the predefined planning pipeline.
- **active_vision_nbv_planning_pipeline.hpp**: This header file defines the `ActiveVisionNbvPlanningPipeline` class, which handles the NBV planning pipeline. 

- **main_predefined_planning_pipeline.cpp**: This source file contains the main entry point for the predefined planning pipeline executable. It initializes the ROS 2 node and starts the predefined planning pipeline. A MultiThreadedExecutor defined in the main node is used to allow multiple nodes to run in separate threads. Then, the main thread is dedicated to running the multi-threaded executor, which spins up all the nodes, allowing them to process their respective tasks concurrently. Next,
the main thread starts the execution of the main function Pipeline.
- **main_nbv_planning_pipeline.cpp**: This source file contains the main entry point for the NBV planning pipeline executable. It initializes the ROS 2 node and starts the NBV planning pipeline. The execution of the node is similar to the one discussed above.
- **predefined_planning_pipeline.cpp**: This source file implements the `ActiveVisionPredefinedPlanningPipeline` class. It includes methods for creating data subscribers, saving data, generating planning poses, and calculating reconstruction metrics. This pipeline executes the active vision procedure, planning the robot in predefined poses, such as zig-zag path.
- **nbv_planning_pipeline.cpp**: This source file implements the `ActiveVisionNbvPlanningPipeline` class. It includes methods for performing ray casting, calculating utility, choosing the next best view, and calculating reconstruction metrics. This pipeline executes the active vision procedure, planning the robot position using NBV planning.
