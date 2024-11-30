```mermaid
classDiagram
    class DroneAgent {
        <<Agent>>
        State:
        - is_flying: bool
        - is_controlled_by_security: bool
        - mission_completed: bool
        - height: float
        - battery: float
        
        Actions:
        + take_off()
        + land()
        + patrol()
        + inspect_area()
        + chase_robber()

        Methods:
        + check_collision()
        + find_safe_position()
        + detect_objects()
        + adjust_height()

        Communication:
        - Protocol: SurveillanceProtocol (INFORM_THREAT)
        - Role: Surveillant
        - Protocol: ControlProtocol (ACCEPT_CONTROL)
        - Role: Responder
    }

    class CameraAgent {
        <<Agent>>
        State:
        - detection_radius: float
        - last_detection_time: datetime
        - detected_agents: Set

        Actions:
        + detect_movement()

        Methods:
        + alert_drone()
        + detect_robbers()

        Communication:
        - Protocol: SurveillanceProtocol (ALERT_THREAT)
        - Role: Observer
        - Protocol: AssistanceProtocol (REQUEST_ASSISTANCE)
        - Role: Alerter
    }

    class SecurityPersonnelAgent {
        <<Agent>>
        State:
        - has_drone_control: bool
        - chasing_robber: bool
        - target_robber: RobberAgent
        - alerted_to_robber: bool
        - movement_speed: float

        Actions:
        + patrol()
        + assess_threat()

        Methods:
        + take_drone_control()
        + release_drone_control()
        + check_collision()
        + respond_to_camera_detection()
        + arrest_robber()

        Communication:
        - Protocol: ControlProtocol (REQUEST_CONTROL)
        - Role: Controller
        - Protocol: PursuitProtocol (COORDINATE_PURSUIT)
        - Role: Pursuer
    }

    class RobberAgent {
        <<Agent>>
        State:
        - is_caught: bool
        - is_spotted: bool

        Actions:
        + move()

        Methods:
        + check_collisions()
        + check_nearby_threats()

        Communication:
        - Protocol: None
        - Role: Target
    }

    class SurveillanceModel {
        <<Environment>>
        - grid_size: int
        - landing_station: Tuple
        - current_step: int

        + setup()
        + calculate_distance()
        + should_initiate_return()
        + move_towards_landing_station()
        + visualize()
    }

    DroneAgent --> SurveillanceModel : operates in
    CameraAgent --> SurveillanceModel : monitors
    SecurityPersonnelAgent --> SurveillanceModel : patrols
    RobberAgent --> SurveillanceModel : exists in

    note as N1
        Interaction Protocols:
        - SurveillanceProtocol
        - ControlProtocol
        - AssistanceProtocol
        - PursuitProtocol
    end note
```
