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
        """Initialize drone parameters"""
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
        
        # Original drone setup
        self.x, self.y = self.model.landing_station
        self.height = 0
        self.battery = 100
        self.is_flying = False
        self.is_controlled_by_security = False
        self.target_height = 5
        self.height_step = 1
        print(f"Drone {self.id} initialized at landing station ({self.x}, {self.y}).")

    def handle_message(self, message):
        if message.protocol == Protocol.CONTROL:
            if message.comm_act == CommunicationAct.REQUEST_CONTROL:
                self.current_role = "Responder"
                self.is_controlled_by_security = message.content['control_status']
                print(f"Drone {self.id} control status changed to: {self.is_controlled_by_security}")
                
        elif message.protocol == Protocol.SURVEILLANCE:
            if message.comm_act == CommunicationAct.ALERT_THREAT:
                target_pos = message.content['location']
                print(f"Drone {self.id} received alert about threat at {target_pos}")


    def inspect_area(self):
        """Inspect the area for suspicious activity."""
        suspicious = random.choice([True, False])  # Simula una detección de amenaza
        if suspicious:
            print(f"Drone {self.id} detected suspicious activity.")
        else:
            print(f"Drone {self.id} found no threats.")
        return suspicious

    def take_off(self):
        """Drone takes off gradually from the landing station"""
        if not self.is_flying:
            print(f"Drone {self.id} is preparing for takeoff.")
            self.is_flying = True  # Marcamos como volando para que inicie
        # Ajustamos la altura en cada paso
        self.adjust_height(self.target_height)
        if self.height == self.target_height:
            print(f"Drone {self.id} has reached cruising altitude ({self.height}).")

    def land(self):
        """Drone lands gradually at the landing station"""
        if self.x == self.model.landing_station[0] and self.y == self.model.landing_station[1]:
            # Ajustamos la altura gradualmente
            if self.height > 0:
                self.adjust_height(0)
                print(f"Drone {self.id} landing... Current height: {self.height}")
            elif self.height == 0:
                self.is_flying = False
                print(f"Drone {self.id} has landed successfully at ({self.x}, {self.y}).")

    def adjust_height(self, target_height):
        """Gradually adjust the drone's height towards the target height"""
        # Asegúrate de que la altura se ajusta solo una vez por llamada
        if self.height != target_height:
            if self.height < target_height:
                self.height += 1
            elif self.height > target_height:
                self.height -= 1
            print(f"Drone {self.id} adjusting height to {self.height} (target: {target_height}).")

    def patrol(self):
        """Patrol the area, moving randomly"""
        if self.is_flying and self.height == self.target_height:  # Asegura que solo patrulle si está volando
            # Movimiento aleatorio en el plano X-Y
            dx, dy = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
            self.x = max(0, min(self.model.grid_size - 1, self.x + dx))
            self.y = max(0, min(self.model.grid_size - 1, self.y + dy))
            
            # Movimiento aleatorio en Z (altura) dentro de un rango permitido (e.g., 2 a 5)
            new_height = self.height + random.choice([-2, -1, 0, 1, 2])
            self.height = max(2, min(5, new_height))  # Restringe la altura entre 2 y 5
            
            # Consumo de batería
            self.battery -= 1
            
            print(f"Drone {self.id} patrolling at ({self.x}, {self.y}, height: {self.height}).")

class Watcher(ap.Agent):
    def setup(self):
        """Initialize watcher parameters"""
        self.x = random.randint(0, self.model.grid_size-1)
        self.y = random.randint(0, self.model.grid_size-1)
        self.height = 4  # Slightly higher than robbers
        self.observation_range = 4  # Larger range than current detection
        self.is_reporting = False
        self.reported_robber = None
        self.patrol_route = self._generate_patrol_route()
        self.current_route_index = 0
        self.communication_cooldown = 0
        self.stress_level = 0  # New attribute to track watcher's psychological state
        self.arrest_range = 1  # Rango de distancia para arrestar
        self.target_robber = None  # Robber que está persiguiendo actualmente
        self.collision_threshold = 1

    def check_collision(self, new_x, new_y):
        """Verifica si hay colisión con otros watchers en la nueva posición"""
        for agent in self.model.agents:
            if agent is not self and isinstance(agent, Watcher):
                distance = np.sqrt((new_x - agent.x)**2 + (new_y - agent.y)**2)
                if distance < self.collision_threshold:
                    return True
        return False

    def get_valid_move(self, target_x, target_y):
        """Obtén un movimiento válido hacia el objetivo o un movimiento aleatorio si está bloqueado."""
        possible_moves = []
        current_distance = np.sqrt((self.x - target_x)**2 + (self.y - target_y)**2)

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_x = max(0, min(self.model.grid_size - 1, self.x + dx))
                new_y = max(0, min(self.model.grid_size - 1, self.y + dy))
                new_distance = np.sqrt((new_x - target_x)**2 + (new_y - target_y)**2)

                # Si el movimiento nos acerca al objetivo y no hay colisión
                if new_distance < current_distance and not self.check_collision(new_x, new_y):
                    possible_moves.append((new_x, new_y))

        if possible_moves:
            return random.choice(possible_moves)  # Elegir uno de los movimientos posibles
        else:
            # Si no hay movimientos válidos, elegir uno aleatorio
            dx, dy = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
            return max(0, min(self.model.grid_size - 1, self.x + dx)), max(0, min(self.model.grid_size - 1, self.y + dy))


    def _generate_patrol_route(self):
        """Generate a predefined patrol route"""
        route = []
        for i in range(0, self.model.grid_size, 3):
            route.append((i, 0))
            route.append((i, self.model.grid_size-1))
        return route

    def advanced_observation(self):
        """Enhanced observation method with collision avoidance"""
        closest_robber = None
        min_distance = float('inf')
        
        for agent in self.model.agents:
            if isinstance(agent, Robber) and not agent.is_caught:
                distance = np.sqrt((self.x - agent.x)**2 + (self.y - agent.y)**2)
                
                # Si el robber está en rango de arresto y no hay colisión
                if distance <= self.arrest_range:
                    self._arrest_robber(agent)
                    return True
                
                # Encontrar el robber más cercano en rango de observación
                elif distance <= self.observation_range and distance < min_distance:
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
        """Patrol the area freely avoiding collisions."""
        if self.target_robber and not self.target_robber.is_caught:
            # Si hay un objetivo, perseguirlo
            self._pursue_robber()
        else:
            # Movimiento aleatorio en las 8 direcciones
            dx, dy = random.choice([
                (1, 0), (-1, 0), (0, 1), (0, -1),  # Movimientos cardinales
                (1, 1), (1, -1), (-1, 1), (-1, -1)  # Movimientos diagonales
            ])
            new_x = max(0, min(self.model.grid_size - 1, self.x + dx))
            new_y = max(0, min(self.model.grid_size - 1, self.y + dy))

            # Evitar colisiones con otros agentes
            if not self.check_collision(new_x, new_y):
                self.x, self.y = new_x, new_y

        print(f"Watcher {self.id} patrolling freely at ({self.x}, {self.y}).")


    def _pursue_robber(self):
        """Persigue al robber objetivo evitando colisiones con otros watchers"""
        if self.target_robber:
            new_x, new_y = self.get_valid_move(self.target_robber.x, self.target_robber.y)
            self.x, self.y = new_x, new_y

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
            if isinstance(agent, (Drone, SecurityPersonnel, Watcher)):
                distance = np.sqrt((self.x - agent.x)**2 + (self.y - agent.y)**2)
                if distance <= self.detection_range:
                    threats.append((agent.x, agent.y))
        return threats

class Camera(CommunicatingAgent):
    def setup(self):
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
        self.x = random.randint(0, self.model.grid_size - 1)
        self.y = random.randint(0, self.model.grid_size - 1)
        self.height = random.randint(3, 8)
        self.detection_radius = 3
        self.last_detection_time = 0
        self.detected_agents = set()

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

class SecurityPersonnel(CommunicatingAgent):
    def setup(self):
        super().setup()
        # Communication setup
        self.roles = ["Controller", "Pursuer"]
        self.current_role = "Controller"
        self.supported_protocols = [Protocol.CONTROL, Protocol.PURSUIT, Protocol.VISION]
        self.supported_acts = [
            CommunicationAct.REQUEST_CONTROL,
            CommunicationAct.COORDINATE_PURSUIT
        ]
        
        # Original security setup
        self.x = random.randint(0, self.model.grid_size-1)
        self.y = random.randint(0, self.model.grid_size-1)
        self.height = 3
        self.has_drone_control = False
        self.chasing_robber = False
        self.target_robber = None
        self.alerted_to_robber = False
        self.movement_speed = 1
        self.collision_threshold = 1

    def handle_message(self, message):
        if message.protocol == Protocol.VISION:
            if message.comm_act == CommunicationAct.REPORT_VISION:
                print(f"Security {self.id} received vision report from Drone {message.sender.id}")
                if 'robber' in message.content:
                    self.take_drone_control(message.sender)
    
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

    def patrol(self):
        """Patrol the area randomly while avoiding collisions"""
        if not self.chasing_robber:
            desired_x = max(0, min(self.model.grid_size-1,
                                self.x + random.choice([-1, 0, 1])))
            desired_y = max(0, min(self.model.grid_size-1,
                                self.y + random.choice([-1, 0, 1])))

            if not self.check_collision(desired_x, desired_y):
                self.x = desired_x
                self.y = desired_y
                print(f"Security {self.id} patrolling at ({self.x}, {self.y}, height: {self.height})")

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

    def release_drone_control(self, drone):
        """Release control of a drone"""
        if drone.is_controlled_by_security:
            drone.is_controlled_by_security = False
            print(f"Security personnel {self.id} released control of Drone {drone.id}.")

    def assess_threat(self, drone):
        """Assess threat detected by drone"""
        threat = drone.inspect_area()
        if threat:
            print(f"SECURITY ALERT: Security personnel {self.id} confirmed a threat detected by Drone {drone.id}.")
            return True
        else:
            print(f"Security personnel {self.id} confirmed no threat detected by Drone {drone.id}.")
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
        self.security = ap.AgentList(self, self.p.num_watchers, SecurityPersonnel)
        self.robbers = ap.AgentList(self, self.p.num_robbers, Robber)
        self.agents = self.drones + self.cameras + self.security + self.robbers + self.watchers
        self.current_step = 0
        self.return_initiated = False
        self.robbers_to_remove = set()

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

    def step(self):
        """Perform one step of the simulation"""
        self.current_step += 1

        for agent in self.agents:
            if isinstance(agent, CommunicatingAgent):
                agent.process_messages()
        
        # Mover robbers activos
        for robber in self.robbers:
            if not robber.is_caught:
                robber.move()
        
        # Actualizar watchers
        for watcher in self.watchers:
            watcher.advanced_observation()
            watcher.patrol()

        # Resto de la lógica del dron...
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
                        security = self.security[0]
                        security.take_drone_control(drone)
                        if not security.assess_threat(drone):
                            security.release_drone_control(drone)

        # Eliminar robbers capturados
        if self.robbers_to_remove:
            self.robbers = ap.AgentList(self, [r for r in self.robbers if r not in self.robbers_to_remove])
            self.agents = self.drones + self.cameras + self.security + self.robbers + self.watchers
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

        security_state = [{
            'id': security.id,
        } for security in self.security]

        return {
            'step': self.current_step,
            'drones': drones_state,
            'cameras': cameras_state,
            'robbers': robbers_state,
            'watchers': watchers_state,
            'security': security_state,
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
    'num_cameras': 3,
    'num_robbers': 2,
    'num_watchers': 2,
    'steps': 50
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
                model.security[0],  # Send to first security personnel
                Protocol.VISION,
                CommunicationAct.REPORT_VISION,
                {
                    'detections': detections,
                    'drone_position': (drone.x, drone.y, drone.height)
                }
            )
            
            # If robbers detected, security will handle through message system
            robber_detections = [d for d in detections if d['type'] == 'robber']
            if robber_detections and not drone.is_controlled_by_security:
                # Security will request control through message system
                model.security[0].take_drone_control(drone)
                
                if model.security[0].assess_threat(drone):
                    nearest_robber = None
                    min_distance = float('inf')
                    
                    for robber in model.robbers:
                        if not robber.is_caught:
                            distance = np.sqrt((drone.x - robber.x)**2 + (drone.y - robber.y)**2)
                            if distance < min_distance:
                                min_distance = distance
                                nearest_robber = robber
                    
                    if nearest_robber:
                        print(f"- Pursuing robber at ({nearest_robber.x}, {nearest_robber.y})")
                        drone.x = max(0, min(model.grid_size - 1, nearest_robber.x))
                        drone.y = max(0, min(model.grid_size - 1, nearest_robber.y))
                        
                        if min_distance <= 1:
                            print(f"- Robber {nearest_robber.id} caught!")
                            nearest_robber.is_caught = True
                            model.robbers_to_remove.add(nearest_robber)
                            model.security[0].release_drone_control(drone)
        
        print("--------------------")
        return jsonify({'status': 'success', 'message': 'Image processed successfully'})
        
    except Exception as e:
        print(f"Error processing image: {str(e)}")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(debug=False)