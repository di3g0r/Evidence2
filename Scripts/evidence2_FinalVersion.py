from flask import Flask, jsonify, request
import agentpy as ap
import numpy as np
import random
from PIL import Image
import io
from ultralytics import YOLO
import cv2
from enum import Enum
import queue

app = Flask(__name__)

class Protocol(Enum):
    SURVEILLANCE = "SurveillanceProtocol"
    CONTROL = "ControlProtocol"
    ASSISTANCE = "AssistanceProtocol"
    PURSUIT = "PursuitProtocol"
    VISION = "VisionProtocol"

class CommunicationAct(Enum):
    INFORM_THREAT = "INFORM_THREAT"
    ALERT_THREAT = "ALERT_THREAT"
    REQUEST_CONTROL = "REQUEST_CONTROL"
    ACCEPT_CONTROL = "ACCEPT_CONTROL"
    REQUEST_ASSISTANCE = "REQUEST_ASSISTANCE"
    COORDINATE_PURSUIT = "COORDINATE_PURSUIT"
    REPORT_VISION = "REPORT_VISION"

class Message:
    def __init__(self, sender, receiver, protocol, comm_act, content):
        self.sender = sender
        self.sender_role = sender.current_role
        self.receiver = receiver
        self.receiver_role = receiver.current_role
        self.protocol = protocol
        self.comm_act = comm_act
        self.content = content
        self.timestamp = 0

class CommunicatingAgent(ap.Agent):
    def setup(self):
        self.message_queue = queue.Queue()
        self.sent_messages = []
        self.roles = []
        self.current_role = None
        
    def send_message(self, receiver, protocol, comm_act, content):
        if protocol in self.supported_protocols and comm_act in self.supported_acts:
            message = Message(self, receiver, protocol, comm_act, content)
            self.sent_messages.append(message)
            receiver.message_queue.put(message)
            print(f"Agent {self.id} as {self.current_role} sent {comm_act.value} to Agent {receiver.id}")
    
    def process_messages(self):
        while not self.message_queue.empty():
            message = self.message_queue.get()
            if message.protocol in self.supported_protocols:
                self.handle_message(message)
            else:
                print(f"Agent {self.id} cannot handle protocol {message.protocol}")

class Drone(CommunicatingAgent):
    def setup(self):
        """Initialize drone parameters."""
        super().setup()
        # Communication setup
        self.roles = ["Surveillant", "Responder"]
        self.current_role = "Surveillant"
        self.supported_protocols = [Protocol.SURVEILLANCE, Protocol.CONTROL, Protocol.VISION]
        self.supported_acts = [
            CommunicationAct.INFORM_THREAT,
            CommunicationAct.ACCEPT_CONTROL,
            CommunicationAct.REPORT_VISION
        ]
        
        # Drone setup
        self.x, self.y = self.model.landing_station
        self.height = 0
        self.battery = 100
        self.is_flying = False
        self.is_controlled_by_security = False
        self.target_height = 5
        self.height_step = 1
        self.target_position = None  # Target position for threats
        #print(f"Drone {self.id} initialized at landing station ({self.x}, {self.y}).")

    def handle_message(self, message):
        """Processes messages received by the drone."""
        if message.protocol == Protocol.SURVEILLANCE and message.comm_act == CommunicationAct.ALERT_THREAT:
            self.target_position = message.content['location']
            print(f"Drone {self.id} received alert about threat at {self.target_position}. Moving to inspect.")
            self.take_off()  # Prepare to move to target

    def move_to_target(self):
        """Moves the drone step-by-step toward the target position."""
        if self.target_position:
            target_x, target_y = self.target_position
            
            # Step-by-step movement in X and Y
            if self.x != target_x:
                self.x += 1 if self.x < target_x else -1
            elif self.y != target_y:
                self.y += 1 if self.y < target_y else -1
            
            print(f"Drone {self.id} moving to ({self.x}, {self.y}). Target: {self.target_position}.")
            
            # If target position is reached
            if self.x == target_x and self.y == target_y:
                print(f"Drone {self.id} reached the target position ({self.x}, {self.y}).")
                self.target_position = None  # Clear target
                self.inspect_area()  # Inspect the area

    def inspect_area(self):
        """Inspects the current area for suspicious activity."""
        for agent in self.model.robbers:  # Verifica si hay robbers cerca
            distance = np.sqrt((self.x - agent.x)**2 + (self.y - agent.y)**2)
            if distance <= 2:  # Supongamos que el rango de detección es 2
                print(f"Drone {self.id} detected suspicious activity near ({agent.x}, {agent.y}).")
                self.notify_security(agent.x, agent.y)
                return True
        print(f"Drone {self.id} found no issues in the area ({self.x}, {self.y}).")
        return False

    def notify_security(self, threat_x, threat_y):
        """Notifies all watchers about a confirmed threat."""
        for watcher in self.model.watchers:  # Notifica a todos los guardias
            self.send_message(
                watcher,
                Protocol.CONTROL,
                CommunicationAct.INFORM_THREAT,
                {'location': (threat_x, threat_y)}
            )
            print(f"Drone {self.id} notified Watcher {watcher.id} about threat at ({threat_x}, {threat_y}).")

    def take_off(self):
        """Takes off the drone toward its target height."""
        if not self.is_flying:
            self.is_flying = True
            #print(f"Drone {self.id} is taking off.")
        self.adjust_height(self.target_height)

    def land(self):
        """Lands the drone at the landing station."""
        if self.x == self.model.landing_station[0] and self.y == self.model.landing_station[1]:
            if self.height > 0:
                self.adjust_height(1)
                #print(f"Drone {self.id} landing... Current height: {self.height}")
            elif self.height == 0:
                self.is_flying = False
                #print(f"Drone {self.id} has landed successfully at ({self.x}, {self.y}).")

    def adjust_height(self, target_height):
        """Gradually adjusts the drone's height towards the target."""
        if self.height != target_height:
            if self.height < target_height:
                self.height += 1
            elif self.height > target_height:
                self.height -= 1
            #print(f"Drone {self.id} adjusting height to {self.height} (target: {target_height}).")

    def patrol(self):
        """Patrols the area, moving randomly."""
        if self.is_flying and self.height == self.target_height:
            dx, dy = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
            self.x = max(0, min(self.model.grid_size - 1, self.x + dx))
            self.y = max(0, min(self.model.grid_size - 1, self.y + dy))
            
            # Random height adjustments within allowed range
            new_height = self.height + random.choice([-2, -1, 0, 1, 2])
            self.height = max(2, min(5, new_height))
            
            # Battery consumption
            self.battery -= 1
            
            #print(f"Drone {self.id} patrolling at ({self.x}, {self.y}, height: {self.height}).")

    def step(self):
        """Performs actions at each simulation step."""
        if self.is_flying:
            if self.target_position:
                self.move_to_target()  # Moves toward the target
            else:
                self.patrol()  # Patrols if no target
        else:
            if self.battery < 20:  # Lands if battery is low
                self.land()

class Watcher(CommunicatingAgent):
    def setup(self):
        super().setup()

        self.roles = ["Surveillant", "Responder"]
        self.current_role = "Surveillant"
        self.supported_protocols = [Protocol.SURVEILLANCE, Protocol.CONTROL, Protocol.VISION]
        self.supported_acts = [
            CommunicationAct.INFORM_THREAT,
            CommunicationAct.ACCEPT_CONTROL,
            CommunicationAct.REPORT_VISION
        ]
        """Initialize watcher parameters with improved dispersion."""

        # Inicializar otros parámetros
        self.height = 4
        self.observation_range = 4
        self.is_reporting = False
        self.reported_robber = None
        self.communication_cooldown = 0
        self.stress_level = 0
        self.arrest_range = 1
        self.target_robber = None
        self.collision_threshold = 1

        # Ruta de patrullaje
        self.patrol_route = self._generate_patrol_route()
        self.current_route_index = 0

        self.current_role = "patrolling"

    def check_collision(self, new_x, new_y):
        """Verifica si hay colisión con otros watchers en la nueva posición"""
        for agent in self.model.watchers:
            if agent is not self and isinstance(agent, Watcher):
                distance = np.sqrt((new_x - agent.x)**2 + (new_y - agent.y)**2)
                if distance < self.collision_threshold:
                    return True
        return False

    def get_valid_move(self, target_x, target_y):
        """Calculate a valid step-by-step move towards the target."""
        best_move = None
        shortest_distance = float('inf')

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            new_x = max(0, min(self.model.grid_size - 1, self.x + dx))
            new_y = max(0, min(self.model.grid_size - 1, self.y + dy))
            distance = np.sqrt((new_x - target_x)**2 + (new_y - target_y)**2)

            # Elegir el movimiento que minimice la distancia y no cause colisiones
            if distance < shortest_distance and not self.check_collision(new_x, new_y):
                best_move = (new_x, new_y)
                shortest_distance = distance

        return best_move if best_move else (self.x, self.y)  # Si no hay movimientos válidos, quedarse en su lugar

    def handle_message(self, message):
        """Processes messages received by the watcher."""
        if message.protocol == Protocol.CONTROL and message.comm_act == CommunicationAct.INFORM_THREAT:
            target_position = message.content['location']
            print(f"Watcher {self.id} received threat alert at {target_position}.")
            self.target_robber = None  # Reinicia cualquier persecución actual
            self.patrol_route = [target_position]  # Reasigna la ruta hacia la amenaza
            self.current_route_index = 0


    def _generate_patrol_route(self):
        """Generate a unique patrol route for each watcher."""
        route = []
        for _ in range(5):  # Generar 5 puntos de patrulla
            while True:
                patrol_x = random.randint(0, self.model.grid_size - 1)
                patrol_y = random.randint(0, self.model.grid_size - 1)

                # Asegurar que los puntos estén suficientemente separados
                if all(np.sqrt((patrol_x - px)**2 + (patrol_y - py)**2) >= 3 for px, py in route):
                    route.append((patrol_x, patrol_y))
                    break
        return route

    def advanced_observation(self):
        """Observe the surroundings for robbers."""
        closest_robber = None
        min_distance = float('inf')
        
        for agent in self.model.agents:
            if isinstance(agent, Robber) and not agent.is_caught:
                distance = np.sqrt((self.x - agent.x)**2 + (self.y - agent.y)**2)
                
                if distance <= self.arrest_range:
                    self._arrest_robber(agent)
                    return True
                
                if distance <= self.observation_range and distance < min_distance:
                    min_distance = distance
                    closest_robber = agent
        
        if closest_robber:
            self.target_robber = closest_robber
            closest_robber.is_spotted = True
            return True
        
        return False

    def _arrest_robber(self, robber):
        """Arresta al robber y notifica al modelo para eliminarlo"""
        robber.is_caught = True
        print(f"Watcher {self.id} arrested Robber {robber.id}!")
        self.model.remove_robber(robber)  # Notifica al modelo para eliminar el robber
        self.target_robber = None

    def patrol(self):
        """Patrol or move toward a reported threat."""
        if self.target_robber and not self.target_robber.is_caught:
            # Perseguir al ladrón
            self._pursue_robber()
        elif self.patrol_route:
            # Seguir patrullando o ir hacia la amenaza
            self._follow_patrol_route()
        else:
            # Movimiento aleatorio si no hay ruta definida
            self._random_patrol()

    def _follow_patrol_route(self):
        """Follow the assigned patrol route."""
        next_position = self.patrol_route[self.current_route_index]
        new_x, new_y = self.get_valid_move(next_position[0], next_position[1])

        if not self.check_collision(new_x, new_y):
            self.x, self.y = new_x, new_y
            self.current_route_index = (self.current_route_index + 1) % len(self.patrol_route)
            print(f"Watcher {self.id} patrolling towards {next_position}. Now at ({self.x}, {self.y}).")

    def _random_patrol(self):
        """Move randomly when no patrol route is assigned."""
        valid_moves = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_x = max(0, min(self.model.grid_size - 1, self.x + dx))
            new_y = max(0, min(self.model.grid_size - 1, self.y + dy))
            if not self.check_collision(new_x, new_y):
                valid_moves.append((new_x, new_y))
        
        if valid_moves:
            self.x, self.y = random.choice(valid_moves)
            print(f"Watcher {self.id} patrolling randomly at ({self.x}, {self.y}).")
        else:
            print(f"Watcher {self.id} is temporarily stuck and remains at ({self.x}, {self.y}).")

    def _pursue_robber(self):
        """Pursue the target robber step by step."""
        new_x, new_y = self.get_valid_move(self.target_robber.x, self.target_robber.y)
        if not self.check_collision(new_x, new_y):
            self.x, self.y = new_x, new_y
            print(f"Watcher {self.id} pursuing Robber at ({self.target_robber.x}, {self.target_robber.y}). Now at ({self.x}, {self.y}).")
        else:
            print(f"Watcher {self.id} blocked while pursuing Robber at ({self.target_robber.x}, {self.target_robber.y}).")

    def take_drone_control(self, drone):
        """Take control of a drone with communication"""
        self.current_role = "Controller"
        self.send_message(
            drone,
            Protocol.CONTROL,
            CommunicationAct.REQUEST_CONTROL,
            {'control_status': True}
        )
        self.has_drone_control = True
        print(f"Security {self.id} requested control of Drone {drone.id}")

    def assess_threat(self, drone):
        """Assess threat detected by drone"""
        threat = drone.inspect_area()
        if threat:
            print(f"SECURITY ALERT: Security personnel {self.id} confirmed a threat detected by Drone {drone.id}.")
            return True
        else:
            print(f"Security personnel {self.id} confirmed no threat detected by Drone {drone.id}.")
            return False
        
    def release_drone_control(self, drone):
        """Release control of a drone"""
        if drone.is_controlled_by_security:
            drone.is_controlled_by_security = False
            print(f"Security personnel {self.id} released control of Drone {drone.id}.")

class Robber(ap.Agent):
    def setup(self):
        """Initialize robber parameters"""
        self.x = random.randint(0, self.model.grid_size-1)
        self.y = random.randint(0, self.model.grid_size-1)
        self.height = 3  # Fixed height for robber
        self.is_caught = False
        self.is_spotted = False
        self.detection_range = 3
        self.collision_threshold = 1

    def check_collision(self, new_x, new_y):
        """Check if moving to new position would cause collision"""
        for agent in self.model.agents:
            if agent is not self and not isinstance(agent, Drone):  # Allow interaction with drones
                distance = np.sqrt(
                    (new_x - agent.x)**2 +
                    (new_y - agent.y)**2 +
                    (self.height - agent.height)**2
                )
                if distance < self.collision_threshold:
                    return True
        return False

    def move(self):
        """Move while trying to avoid detection and collisions"""
        if not self.is_caught:
            nearby_threats = self.check_nearby_threats()

            if nearby_threats:
                # Intentar alejarse de las amenazas
                avg_threat_x = sum(t[0] for t in nearby_threats) / len(nearby_threats)
                avg_threat_y = sum(t[1] for t in nearby_threats) / len(nearby_threats)
                
                desired_x = self.x + (-1 if self.x < avg_threat_x else 1)
                desired_y = self.y + (-1 if self.y < avg_threat_y else 1)
                
                desired_x = max(0, min(self.model.grid_size-1, desired_x))
                desired_y = max(0, min(self.model.grid_size-1, desired_y))
            else:
                desired_x = max(0, min(self.model.grid_size-1,
                                    self.x + random.choice([-1, 0, 1])))
                desired_y = max(0, min(self.model.grid_size-1,
                                    self.y + random.choice([-1, 0, 1])))

            if not self.check_collision(desired_x, desired_y):
                self.x = desired_x
                self.y = desired_y

    def check_nearby_threats(self):
        """Check for nearby drones and security personnel"""
        threats = []
        for agent in self.model.agents:
            if isinstance(agent, (Drone, Watcher)):
                distance = np.sqrt((self.x - agent.x)**2 + (self.y - agent.y)**2)
                if distance <= self.detection_range:
                    threats.append((agent.x, agent.y))
        return threats

class Camera(CommunicatingAgent):
    def setup(self):
        """Initialize fixed camera parameters."""
        super().setup()
        # Communication setup
        self.roles = ["Observer", "Alerter"]
        self.current_role = "Observer"
        self.supported_protocols = [Protocol.SURVEILLANCE, Protocol.ASSISTANCE]
        self.supported_acts = [
            CommunicationAct.ALERT_THREAT,
            CommunicationAct.REQUEST_ASSISTANCE
        ]
        
        # Original camera setup
        self.x, self.y = self.place_on_edge()  # Cámaras ubicadas en los bordes
        self.height = 15  # Altura fija para la cámara
        self.detection_radius = 3  # Radio de detección
        #print(f"Camera {self.id} initialized at position ({self.x}, {self.y}, height: {self.height}).")

    def place_on_edge(self):
        """Place cameras on the edges of the grid, cycling through sides."""
        grid_size = self.model.grid_size

        # Determine placement based on camera ID
        camera_index = self.id  # Unique ID of the camera
        side = camera_index % 4  # Cycle through the 4 sides (0: top, 1: right, 2: bottom, 3: left)

        # Calculate position for the current side
        if side == 0:  # Top edge
            x = (camera_index // 4 + 1) * (grid_size // (self.p.num_cameras // 4 + 1))
            y = 0
        elif side == 1:  # Right edge
            x = grid_size - 1
            y = (camera_index // 4 + 1) * (grid_size // (self.p.num_cameras // 4 + 1))
        elif side == 2:  # Bottom edge
            x = (camera_index // 4 + 1) * (grid_size // (self.p.num_cameras // 4 + 1))
            y = grid_size - 1
        elif side == 3:  # Left edge
            x = 0
            y = (camera_index // 4 + 1) * (grid_size // (self.p.num_cameras // 4 + 1))

        # Ensure the coordinates are within grid bounds
        x = min(x, grid_size - 1)
        y = min(y, grid_size - 1)

        return x, y


    def handle_message(self, message):
        """Handle incoming messages."""
        if message.protocol == Protocol.SURVEILLANCE:
            if message.comm_act == CommunicationAct.REQUEST_ASSISTANCE:
                print(f"Camera {self.id} received request for assistance at {message.content['location']}.")
                # Aquí podrías implementar lógica adicional si la cámara requiere enviar más datos.

    def notify_drone(self, drone, threat_location):
        """Envía una alerta al dron sobre una posible amenaza."""
        self.send_message(
            drone,
            Protocol.SURVEILLANCE,
            CommunicationAct.ALERT_THREAT,
            {'location': threat_location}
        )
        print(f"Camera {self.id} alerted Drone {drone.id} about a threat at {threat_location}.")

    def detect_movement(self, drone):
        """Detect if the drone is within the detection radius and communicate"""
        distance = np.sqrt((self.x - drone.x)**2 + (self.y - drone.y)**2)
        if distance <= self.detection_radius:
            print(f"Camera {self.id} detected movement of Drone {drone.id}")
            self.current_role = "Alerter"
            self.send_message(
                drone,
                Protocol.SURVEILLANCE,
                CommunicationAct.ALERT_THREAT,
                {'location': (self.x, self.y)}
            )
            return True
        return False

class SurveillanceModel(ap.Model):
    def setup(self):
        """Initialize the simulation environment"""
        self.space = ap.Space(self, shape=[self.p.grid_size, self.p.grid_size])
        self.grid_size = self.p.grid_size
        self.landing_station = (self.grid_size // 2, self.grid_size // 2)
        
        # Inicializar watchers en posiciones no superpuestas
        self.watchers = ap.AgentList(self, self.p.num_watchers, Watcher)
        self._position_watchers()

        self.drones = ap.AgentList(self, self.p.num_drones, Drone)
        self.cameras = ap.AgentList(self, self.p.num_cameras, Camera)
        self.robbers = ap.AgentList(self, self.p.num_robbers, Robber)
        self.agents = self.drones + self.cameras + self.robbers + self.watchers
        self.current_step = 0
        self.return_initiated = False
        self.robbers_to_remove = set()
        self.movement_log = []  # Para registrar los movimientos detectados por las cámaras

    def detect_camera_movement(self):
        """Detecta movimientos de ladrones dentro del rango de las cámaras."""
        for camera in self.cameras:
            for robber in self.robbers:
                if not robber.is_caught:
                    distance = ((camera.x - robber.x)**2 + (camera.y - robber.y)**2)**0.5
                    if distance <= camera.detection_radius:
                        if hasattr(robber, 'previous_position'):
                            if (robber.x, robber.y) != robber.previous_position:
                                self.movement_log.append(f"Step {self.current_step}: Camera {camera.id} detected Robber {robber.id} moving from {robber.previous_position} to ({robber.x}, {robber.y})")
                        robber.previous_position = (robber.x, robber.y)

    def _position_watchers(self):
        """Posiciona los watchers inicialmente sin superposiciones"""
        occupied_positions = set()
        
        for watcher in self.watchers:
            while True:
                x = random.randint(0, self.grid_size-1)
                y = random.randint(0, self.grid_size-1)
                
                # Verificar si la posición está lo suficientemente lejos de otras posiciones ocupadas
                valid_position = True
                for pos_x, pos_y in occupied_positions:
                    if np.sqrt((x - pos_x)**2 + (y - pos_y)**2) < watcher.collision_threshold:
                        valid_position = False
                        break
                
                if valid_position:
                    watcher.x = x
                    watcher.y = y
                    occupied_positions.add((x, y))
                    break

    def remove_robber(self, robber):
        """Marca un robber para ser eliminado al final del paso"""
        self.robbers_to_remove.add(robber)

    # Imprimir logs de movimiento
    def print_movement_logs(self):
        """Imprime los logs de movimiento y limpia el registro."""
        for log in self.movement_log:
            print(log)
        self.movement_log.clear()

    def step(self):
        """Realiza un paso de simulación"""
        self.current_step += 1

        self.detect_camera_movement()

        self.print_movement_logs()

        # Procesar mensajes de agentes comunicantes
        for agent in self.agents:
            if isinstance(agent, CommunicatingAgent):
                agent.process_messages()

        # Movimiento de robbers activos
        for robber in self.robbers:
            if not robber.is_caught:
                robber.move()

        # Acción de los watchers
        for watcher in self.watchers:
            # Observan su entorno y patrullan si no están persiguiendo
            watcher.advanced_observation()  
            watcher.patrol()

        # Control del dron
        drone = self.drones[0]
        if self.should_initiate_return(drone):
            self.return_initiated = True
            self.move_towards_landing_station(drone)
            if drone.x == self.landing_station[0] and drone.y == self.landing_station[1]:
                drone.land()
        else:
            if not drone.is_flying:
                drone.take_off()
            elif drone.is_flying and drone.height < drone.target_height:
                drone.adjust_height(drone.target_height)
            elif drone.is_flying and drone.height == drone.target_height:
                drone.patrol()
                for camera in self.cameras:
                    if camera.detect_movement(drone):
                        watcher = self.watchers[0]
                        watcher.take_drone_control(drone)
                        if not watcher.assess_threat(drone):
                            watcher.release_drone_control(drone)

        # Remover robbers atrapados
        for robber in self.robbers_to_remove:
            self.robbers.remove(robber)
        self.robbers_to_remove.clear()


    def get_state(self):
        """Return the current state of the simulation."""
        robbers_state = [{
            'id': robber.id,
            'x': robber.x,
            'y': robber.y,
            'height': robber.height,
            'is_caught': robber.is_caught,
            'is_spotted': robber.is_spotted
        } for robber in self.robbers]

        watchers_state = [{
            'id': watcher.id,
            'x': watcher.x,
            'y': watcher.y,
            'height': watcher.height,
            'stress_level': watcher.stress_level,
            'is_reporting': watcher.is_reporting
        } for watcher in self.watchers]

        drones_state = [{
            'id': drone.id,
            'x': drone.x,
            'y': drone.y,
            'height': drone.height,
            'battery': drone.battery,
            'is_flying': drone.is_flying,
            'controlled_by_security': drone.is_controlled_by_security
        } for drone in self.drones]

        cameras_state = [{
            'id': camera.id,
            'x': camera.x,
            'y': camera.y,
            'detection_radius': camera.detection_radius
        } for camera in self.cameras]

        return {
            'step': self.current_step,
            'drones': drones_state,
            'cameras': cameras_state,
            'robbers': robbers_state,
            'watchers': watchers_state,
            'landing_stations': [self.landing_station],  # Enviamos las coordenadas de la estación de aterrizaje
            'return_initiated': self.return_initiated
        }


    def move_towards_landing_station(self, drone):
        """Move the drone gradually towards the landing station"""
        if drone.x < self.landing_station[0]:
            drone.x += 1
        elif drone.x > self.landing_station[0]:
            drone.x -= 1
        elif drone.y < self.landing_station[1]:
            drone.y += 1
        elif drone.y > self.landing_station[1]:
            drone.y -= 1
        else:
            drone.adjust_height(0)  # Adjust height only when at the station

    def should_initiate_return(self, drone):
        """
        Determina si el dron debe iniciar el regreso al punto de aterrizaje
        basándose en la distancia y el tiempo restante.
        """
        distance = abs(drone.x - self.landing_station[0]) + abs(drone.y - self.landing_station[1])
        steps_remaining = self.p.steps - self.current_step  # Pasos restantes en la simulación
        return steps_remaining <= distance + 2  # Regresa True si no hay tiempo suficiente


parameters = {
    'grid_size': 20,
    'num_drones': 1,
    'num_cameras': 4,
    'num_robbers': 2,
    'num_watchers': 2,
    'steps': 75
}

model = SurveillanceModel(parameters)
model.setup()

@app.route('/simulation-state', methods=['GET'])
def get_simulation_state():
    model.step()
    state = model.get_state()
    return jsonify(state)

@app.route('/reset', methods=['POST'])
def reset_simulation():
    global model
    model = SurveillanceModel(parameters)
    model.setup()
    model.current_step = 0  # Reinicia correctamente el contador
    return jsonify({"message": "Simulation reset successful"})

def generate_scene_description(robber_contours, police_contours, drone):
    description = "Drone's Visual Analysis:\n"
    
    # Describe drone's position
    description += f"I am currently flying at position ({drone.x}, {drone.y}) at a height of {drone.height} units. "
    
    # Describe what's in view
    if len(robber_contours) == 0 and len(police_contours) == 0:
        description += "My cameras are not detecting any significant movement or suspicious activity in the area.\n"
    else:
        if len(robber_contours) > 0:
            if len(robber_contours) == 1:
                description += "I have identified a suspicious individual wearing a black cap. "
            else:
                description += f"I can see {len(robber_contours)} individuals with black caps, potentially suspects. "
        
        if len(police_contours) > 0:
            if len(police_contours) == 1:
                description += "There is one police officer in navy uniform within my field of view. "
            else:
                description += f"I can see {len(police_contours)} police officers patrolling the area. "
        
        # Add relative positions if detected
        if len(robber_contours) > 0:
            robber_positions = [cv2.boundingRect(c) for c in robber_contours]
            left_side = any(x < 128 for x, _, _, _ in robber_positions)
            right_side = any(x >= 128 for x, _, _, _ in robber_positions)
            if left_side and right_side:
                description += "\nSuspects are spread across my field of view. "
            elif left_side:
                description += "\nSuspects are primarily on the left side of my view. "
            else:
                description += "\nSuspects are primarily on the right side of my view. "
        
        # Add action description
        if not drone.is_controlled_by_security:
            description += "\nMaintaining routine surveillance pattern."
        else:
            description += "\nCurrently under security control, tracking suspicious activity."
    
    return description

def generate_scene_description(robber_contours, police_contours, camera_id):
    description = f"Camera {camera_id} Visual Analysis:\n"
    
    # Descripción de lo que está viendo la cámara
    if len(robber_contours) == 0 and len(police_contours) == 0:
        description += "La cámara no está detectando ningún movimiento significativo o actividad sospechosa en la zona.\n"
    else:
        if len(robber_contours) > 0:
            if len(robber_contours) == 1:
                description += "He identificado a un individuo sospechoso con una gorra negra. "
            else:
                description += f"Veo {len(robber_contours)} individuos con gorras negras, posiblemente sospechosos. "
        
        if len(police_contours) > 0:
            if len(police_contours) == 1:
                description += "Hay un oficial de policía con uniforme azul marino en mi campo de visión. "
            else:
                description += f"Veo {len(police_contours)} oficiales de policía patrullando la zona. "
        
        # Descripción de la posición relativa de los sospechosos
        if len(robber_contours) > 0:
            robber_positions = [cv2.boundingRect(c) for c in robber_contours]
            left_side = any(x < 128 for x, _, _, _ in robber_positions)
            right_side = any(x >= 128 for x, _, _, _ in robber_positions)
            if left_side and right_side:
                description += "\nLos sospechosos están distribuidos en mi campo de visión. "
            elif left_side:
                description += "\nLos sospechosos están principalmente a la izquierda de mi vista. "
            else:
                description += "\nLos sospechosos están principalmente a la derecha de mi vista. "
        
        # Acción en curso
        description += "\nMonitoreo en curso de la zona. "
    
    return description

def process_image(image_bytes):
    img_array = np.frombuffer(image_bytes, dtype=np.uint8)
    img_array = img_array.reshape((256, 256, 3))
    hsv = cv2.cvtColor(img_array, cv2.COLOR_RGB2HSV)

    # Rangos de colores
    robber_black_lower, robber_black_upper = np.array([0, 0, 0]), np.array([180, 255, 30])
    robber_brown_lower, robber_brown_upper = np.array([10, 50, 50]), np.array([20, 255, 200])
    police_blue_lower, police_blue_upper = np.array([100, 50, 50]), np.array([130, 255, 150])
    police_yellow_lower, police_yellow_upper = np.array([20, 100, 100]), np.array([30, 255, 255])

    # Generar máscaras
    robber_mask = cv2.bitwise_and(
        cv2.inRange(hsv, robber_black_lower, robber_black_upper),
        cv2.inRange(hsv, robber_brown_lower, robber_brown_upper)
    )
    police_mask = cv2.bitwise_and(
        cv2.inRange(hsv, police_blue_lower, police_blue_upper),
        cv2.inRange(hsv, police_yellow_lower, police_yellow_upper)
    )

    # Procesar máscaras
    kernel = np.ones((5, 5), np.uint8)
    robber_mask = cv2.dilate(robber_mask, kernel, iterations=2)
    police_mask = cv2.dilate(police_mask, kernel, iterations=2)

    # Detectar contornos
    min_contour_area = 50
    robber_contours, _ = cv2.findContours(robber_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    police_contours, _ = cv2.findContours(police_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    robber_contours = [c for c in robber_contours if cv2.contourArea(c) > min_contour_area]
    police_contours = [c for c in police_contours if cv2.contourArea(c) > min_contour_area]

    return robber_contours, police_contours

@app.route('/process-vision', methods=['POST'])
def process_vision():
    if 'image' not in request.files:
        return jsonify({'error': 'No image provided'}), 400
        
    image_file = request.files['image']
    image_bytes = image_file.read()
    
    try:
        img_array = np.frombuffer(image_bytes, dtype=np.uint8)
        img_array = img_array.reshape((256, 256, 3))
        hsv = cv2.cvtColor(img_array, cv2.COLOR_RGB2HSV)
        
        # Define color ranges for robber (black cap and brown skin)
        robber_black_lower = np.array([0, 0, 0])
        robber_black_upper = np.array([180, 255, 30])
        
        robber_brown_lower = np.array([10, 50, 50])
        robber_brown_upper = np.array([20, 255, 200])
        
        robber_black_mask = cv2.inRange(hsv, robber_black_lower, robber_black_upper)
        robber_brown_mask = cv2.inRange(hsv, robber_brown_lower, robber_brown_upper)
        
        # Detect robber where we see both black and brown
        robber_kernel = np.ones((5,5), np.uint8)
        robber_black_mask = cv2.dilate(robber_black_mask, robber_kernel, iterations=2)
        robber_brown_mask = cv2.dilate(robber_brown_mask, robber_kernel, iterations=2)
        robber_mask = cv2.bitwise_and(robber_black_mask, robber_brown_mask)
        
        # Police detection (navy blue and yellow)
        police_blue_lower = np.array([100, 50, 50])
        police_blue_upper = np.array([130, 255, 150])
        police_yellow_lower = np.array([20, 100, 100])
        police_yellow_upper = np.array([30, 255, 255])
        
        police_blue_mask = cv2.inRange(hsv, police_blue_lower, police_blue_upper)
        police_yellow_mask = cv2.inRange(hsv, police_yellow_lower, police_yellow_upper)
        
        police_kernel = np.ones((5,5), np.uint8)
        police_blue_mask = cv2.dilate(police_blue_mask, police_kernel, iterations=2)
        police_yellow_mask = cv2.dilate(police_yellow_mask, police_kernel, iterations=2)
        police_mask = cv2.bitwise_and(police_blue_mask, police_yellow_mask)
        
        # Find contours
        min_contour_area = 50
        robber_contours, _ = cv2.findContours(robber_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        robber_contours = [c for c in robber_contours if cv2.contourArea(c) > min_contour_area]
        
        police_contours, _ = cv2.findContours(police_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        police_contours = [c for c in police_contours if cv2.contourArea(c) > min_contour_area]
        
        # Generate scene description
        drone = model.drones[0]
        scene_description = generate_scene_description(robber_contours, police_contours, drone)
        print("\n" + scene_description)
        
        # Create detection report
        detections = []
        if len(robber_contours) > 0:
            for i, contour in enumerate(robber_contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'type': 'robber',
                    'id': i+1,
                    'position': (x, y),
                    'area': area
                })
                print(f"- Robber {i+1} detected: Area = {area:.2f}, Position = ({x}, {y})")
        
        if len(police_contours) > 0:
            for i, contour in enumerate(police_contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'type': 'police',
                    'id': i+1,
                    'position': (x, y),
                    'area': area
                })
                print(f"- Police {i+1} detected: Area = {area:.2f}, Position = ({x}, {y})")
        
        # Use communication system for detections
        if detections and model.drones:
            drone = model.drones[0]
            drone.send_message(
                model.watchers[0],  # Send to first security personnel
                Protocol.VISION,
                CommunicationAct.REPORT_VISION,
                {
                    'detections': detections,
                    'drone_position': (drone.x, drone.y, drone.height)
                }
            )
                    
            # for robber in model.robbers:
            #     if not robber.is_caught:
            #         distance = np.sqrt((drone.x - robber.x)**2 + (drone.y - robber.y)**2)
            #         if distance < min_distance:
            #             min_distance = distance
            #             nearest_robber = robber
                    
            # if nearest_robber:
            #     print(f"- Pursuing robber at ({nearest_robber.x}, {nearest_robber.y})")
            #     drone.x = max(0, min(model.grid_size - 1, nearest_robber.x))
            #     drone.y = max(0, min(model.grid_size - 1, nearest_robber.y))
                    
            #     if min_distance <= 1:
            #         print(f"- Robber {nearest_robber.id} caught!")
            #         nearest_robber.is_caught = True
            #         model.robbers_to_remove.add(nearest_robber)
            #         model.watchers[0].release_drone_control(drone)
        
        print("--------------------")
        return jsonify({'status': 'success', 'message': 'Image processed successfully'})
        
    except Exception as e:
        print(f"Error processing image: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/camera-vision', methods=['POST'])
def process_vision_camera():
    if 'camera_id' not in request.form or 'image' not in request.files:
        return jsonify({'error': 'Missing camera ID or image'}), 400

    camera_id = request.form['camera_id']
    image_bytes = request.files['image'].read()
    #print(f"Image bytes hash: {hash(image_bytes)}")
    robber_contours, police_contours = process_image(image_bytes)

    # Resto de la lógica específica para cámaras
    scene_description = generate_scene_description(robber_contours, police_contours, camera_id)
    #print("\n" + scene_description)
    return jsonify({'status': 'success', 'message': 'Image processed successfully', 'description': scene_description})

if __name__ == '__main__':
    app.run(debug=False)