using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

public class DroneSimulationVisualizer : MonoBehaviour
{
    [Header("API Configuration")]
    public string apiUrl = "http://127.0.0.1:5000";

    [Header("Prefabs")]
    public GameObject dronePrefab;
    public GameObject cameraPrefab;
    public GameObject robberPrefab;
    public GameObject securityPrefab; // Prefab para el personal de seguridad
    public GameObject landingZonePrefab;

    [Header("Landing Zone Settings")]
    private List<GameObject> landingZoneObjects = new List<GameObject>();

    [Header("Grid Settings")]
    public GameObject gridParent;
    public Material gridLineMaterial;
    public Material detectionRangeMaterial; // Material para las esferas de detecci n
    public float cellSize = 1.0f;
    private int gridSize = 20;

    private GameObject droneObject;
    private List<GameObject> cameraObjects = new List<GameObject>();
    private List<GameObject> robberObjects = new List<GameObject>();
    private List<GameObject> securityObjects = new List<GameObject>();

    void Start()
    {
        InitializeScene();
        StartCoroutine(FetchSimulationStateLoop());
    }

    void InitializeScene()
    {
        Debug.Log("Starting InitializeScene");
    
        // Check if prefab is assigned
        if (dronePrefab != null)
        {
            Debug.Log($"Drone prefab is assigned: {dronePrefab.name}");
            
            // Check prefab for DroneVision before instantiation
            DroneVision prefabVision = dronePrefab.GetComponent<DroneVision>();
            if (prefabVision != null)
            {
                Debug.Log("DroneVision found on prefab");
            }
            else
            {
                Debug.LogError("DroneVision NOT found on prefab!");
            }

            // Instantiate the drone
            droneObject = Instantiate(dronePrefab, Vector3.zero, Quaternion.identity);
            Debug.Log($"Drone instantiated: {droneObject.name}");
            
            // Check instantiated object
            DroneVision vision = droneObject.GetComponent<DroneVision>();
            if (vision != null)
            {
                Debug.Log("DroneVision component found on instantiated drone");
            }
            else
            {
                Debug.LogError("DroneVision component NOT found on instantiated drone!");
                
                // List all components to debug
                Component[] components = droneObject.GetComponents<Component>();
                Debug.Log("Components on instantiated drone:");
                foreach (Component comp in components)
                {
                    Debug.Log($"- {comp.GetType().Name}");
                }
            }
            
            // Check camera
            Camera cam = droneObject.GetComponentInChildren<Camera>();
            if (cam != null)
            {
                Debug.Log("Camera found in drone children");
            }
            else
            {
                Debug.LogError("No Camera found in drone children!");
            }
        }
        else
        {
            Debug.LogError("Drone Prefab is not assigned in inspector!");
        }

        GenerateGrid();
    }

    IEnumerator FetchSimulationStateLoop()
    {
        while (true)
        {
            yield return StartCoroutine(FetchSimulationState());
            yield return new WaitForSeconds(0.5f);
        }
    }

    IEnumerator FetchSimulationState()
    {
        UnityWebRequest request = UnityWebRequest.Get($"{apiUrl}/simulation-state");
        yield return request.SendWebRequest();

        if (request.result == UnityWebRequest.Result.Success)
        {
            string jsonResponse = request.downloadHandler.text;
            Debug.Log($"Raw JSON Response: {jsonResponse}");

            try
            {
                SimulationState state = JsonConvert.DeserializeObject<SimulationState>(jsonResponse);

                if (state != null)
                {
                    gridSize = state.grid_size;
                    UpdateVisualization(state);
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"JSON Parsing Error: {ex.Message}");
            }
        }
        else
        {
            Debug.LogError($"Request Error: {request.error}");
        }
    }

    void UpdateVisualization(SimulationState state)
    {
        // Actualizaci n del dron
        UpdateDrones(state.drones);

        // Actualizaci n de c maras
        UpdateCameras(state.cameras);

        // Actualizaci n de ladrones
        UpdateRobbers(state.robbers);

        // Actualizaci n de personal de seguridad
        UpdateSecurity(state.watchers);

        // Actualizaci n de zonas de aterrizaje
        UpdateLandingZones(state.landing_stations);
    }

    void UpdateDrones(DroneState[] drones)
    {
        if (drones != null && drones.Length > 0)
        {
            var firstDrone = drones[0];
            if (droneObject != null && (firstDrone.is_flying || firstDrone.height > 0))
            {
                droneObject.transform.position = new Vector3(
                    firstDrone.x * cellSize,
                    firstDrone.height * cellSize,
                    firstDrone.y * cellSize
                );
                Debug.Log($"Drone Position Updated: ({firstDrone.x}, {firstDrone.y}, {firstDrone.height})");
            }
        }
    }

    void UpdateCameras(CameraState[] cameras)
    {
        foreach (GameObject cam in cameraObjects)
        {
            Destroy(cam);
        }
        cameraObjects.Clear();

        if (cameras != null)
        {
            foreach (var cam in cameras)
            {
                if (cameraPrefab != null)
                {
                    GameObject cameraObject = Instantiate(
                        cameraPrefab,
                        new Vector3(cam.x * cellSize, cellSize / 2, cam.y * cellSize),
                        Quaternion.identity
                    );
                    cameraObjects.Add(cameraObject);

                    // Genera una esfera proporcional para visualizar el rango de detecci n
                    if (detectionRangeMaterial != null)
                    {
                        GameObject detectionSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        detectionSphere.transform.position = new Vector3(cam.x * cellSize, 0.1f, cam.y * cellSize);
                        detectionSphere.transform.localScale = new Vector3(
                            cam.detection_radius * 2 * cellSize,
                            0.1f,
                            cam.detection_radius * 2 * cellSize
                        ); // Aplanada para no cubrir la vista
                        detectionSphere.GetComponent<Renderer>().material = detectionRangeMaterial;
                        Destroy(detectionSphere.GetComponent<Collider>()); // Elimina el collider
                        Destroy(detectionSphere, 1.0f); // Destruye la esfera despu s de 1 segundo
                    }
                }
                else
                {
                    Debug.LogError("Camera Prefab is not assigned.");
                }
            }
        }
    }

    void UpdateRobbers(RobberState[] robbers)
    {
        foreach (GameObject robber in robberObjects)
        {
            Destroy(robber);
        }
        robberObjects.Clear();

        if (robbers != null)
        {
            foreach (var robber in robbers)
            {
                // Solo crear el objeto si el ladr n no ha sido atrapado
                if (robberPrefab != null && !robber.is_caught)
                {
                    GameObject robberObject = Instantiate(
                        robberPrefab,
                        new Vector3(robber.x * cellSize, 1, robber.y * cellSize),
                        Quaternion.identity
                    );
                    robberObjects.Add(robberObject);
                    Debug.Log($"Thief Spawned at ({robber.x}, {robber.y}).");
                }
                else if (robber.is_caught)
                {
                    Debug.Log($"Thief at ({robber.x}, {robber.y}) has been caught and removed.");
                }
            }
        }
    }

    void UpdateSecurity(WatcherState[] watchers)
    {
        foreach (GameObject sec in securityObjects)
        {
            Destroy(sec);
        }
        securityObjects.Clear();

        if (watchers != null)
        {
            foreach (var guard in watchers)
            {
                if (securityPrefab != null)
                {
                    GameObject securityObject = Instantiate(
                        securityPrefab,
                        new Vector3(guard.x * cellSize, 1, guard.y * cellSize),
                        Quaternion.identity
                    );
                    securityObjects.Add(securityObject);
                    Debug.Log($"Security Guard at ({guard.x}, {guard.y})");

                    // Si quieres que se muevan, aseg rate de actualizar sus posiciones aqu 
                }
                else
                {
                    Debug.LogError("Security Prefab is not assigned.");
                }
            }
        }
    }

    void UpdateLandingZones(int[][] landingStations)
    {
        foreach (GameObject zone in landingZoneObjects)
        {
            Destroy(zone);
        }
        landingZoneObjects.Clear();

        if (landingStations != null)
        {
            foreach (var station in landingStations)
            {
                if (landingZonePrefab != null)
                {
                    GameObject landingZoneObject = Instantiate(
                        landingZonePrefab,
                        new Vector3(station[0] * cellSize, 0, station[1] * cellSize),
                        Quaternion.identity
                    );
                    landingZoneObjects.Add(landingZoneObject);
                    Debug.Log($"Landing Zone Spawned at ({station[0]}, {station[1]}).");
                }
                else
                {
                    Debug.LogError("Landing Zone Prefab is not assigned.");
                }
            }
        }
    }

    void GenerateGrid()
    {
        foreach (Transform child in gridParent.transform)
        {
            Destroy(child.gameObject);
        }

        GameObject gridPlane = GameObject.CreatePrimitive(PrimitiveType.Plane);
        gridPlane.transform.SetParent(gridParent.transform);
        gridPlane.transform.position = new Vector3(gridSize * cellSize / 2, 0, gridSize * cellSize / 2);
        gridPlane.transform.localScale = new Vector3(gridSize * cellSize / 10, 1, gridSize * cellSize / 10);

        if (gridLineMaterial != null)
        {
            Renderer renderer = gridPlane.GetComponent<Renderer>();
            renderer.material = gridLineMaterial;
        }
    }

    [System.Serializable]
    public class SimulationState
    {
        public int grid_size;
        public int[][] landing_stations;
        public DroneState[] drones;
        public CameraState[] cameras;
        public RobberState[] robbers;
        public WatcherState[] watchers;
    }

    [System.Serializable]
    public class DroneState
    {
        public int id;
        public float x, y, height;
        public float battery;
        public bool is_flying;
        public bool controlled_by_security;
    }

    [System.Serializable]
    public class CameraState
    {
        public int id;
        public float x, y;
        public float detection_radius;
    }

    [System.Serializable]
    public class RobberState
    {
        public int id;
        public float x, y;
        public bool is_caught;
        public bool is_spotted;
    }

    [System.Serializable]
    public class WatcherState
    {
        public int id;
        public float x, y;
    }
}
