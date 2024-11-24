from flask import Flask, jsonify
import agentpy as ap
import numpy as np
import random

app = Flask(__name__)

class Drone(ap.Agent):
    def setup(self):
        """Initialize drone parameters"""
        self.x, self.y = self.model.landing_station
        self.height = 0  # At ground level
        self.battery = 100
        self.is_flying = False
        self.is_controlled_by_security = False
        self.target_height = 5
        self.height_step = 1  # Step for height adjustments
        print(f"Drone {self.id} initialized at landing station ({self.x}, {self.y}).")


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

class Camera(ap.Agent):
    def setup(self):
        """Initialize camera at a random location"""
        self.x = random.randint(0, self.model.grid_size - 1)
        self.y = random.randint(0, self.model.grid_size - 1)
        self.height = random.randint(3, 8)  # Random height between 3-8 units
        self.detection_radius = 3
        self.last_detection_time = 0
        self.detected_agents = set()

    def detect_movement(self, drone):
        """Detect if the drone is within the detection radius"""
        distance = np.sqrt((self.x - drone.x)**2 + (self.y - drone.y)**2)
        if distance <= self.detection_radius:
            print(f"Camera {self.id} detected movement of Drone {drone.id}.")
            return True
        return False

class SecurityPersonnel(ap.Agent):
    def setup(self):
        """Initialize security personnel parameters"""
        self.x = random.randint(0, self.model.grid_size-1)
        self.y = random.randint(0, self.model.grid_size-1)
        self.height = 3  # Fixed height for security personnel
        self.has_drone_control = False
        self.chasing_robber = False
        self.target_robber = None
        self.alerted_to_robber = False
        self.movement_speed = 1
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
        """Take control of a drone"""
        if not drone.is_controlled_by_security:
            drone.is_controlled_by_security = True
            print(f"Security personnel {self.id} took control of Drone {drone.id}.")

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
    'steps': 100
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

if __name__ == '__main__':
    app.run(debug=False)