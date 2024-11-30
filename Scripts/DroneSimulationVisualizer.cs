using Newtonsoft.Json;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UIElements;

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
    private GameObject landingZoneObject;

    [Header("Wall Settings")]
    public GameObject wallPrefab; // Prefab para las paredes
    public float wallHeight = 5.0f; // Altura de las paredes

    [Header("Landing Zone Settings")]
    private List<GameObject> landingZoneObjects = new List<GameObject>();

    [Header("Post Settings")]
    public GameObject postPrefab; // Prefab para los postes
    public float postHeight = 5.0f; // Altura de los postes

    [Header("Scenery")]
    public GameObject[] sceneryPrefabs;
    public int numberOfSceneryObjects = 20;

    [Header("Grid Settings")]
    public GameObject gridParent;
    public Material gridLineMaterial;
    public Material detectionRangeMaterial; // Material para las esferas de detecci�n
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

        if (dronePrefab == null)
        {
            Debug.LogError("Drone Prefab is not assigned in inspector!");
            return;
        }

        // Instanciar el dron
        droneObject = Instantiate(dronePrefab, Vector3.zero, Quaternion.identity);
        Debug.Log($"Drone instantiated: {droneObject.name}");

        GenerateGrid();
        AddRandomScenery();
        //GeneratePosts();
        //GenerateWalls(); // Generar las paredes después de la cuadrícula
    }
    /*
    void GenerateWalls()
    {
        if (wallPrefab == null)
        {
            Debug.LogError("Wall Prefab is not assigned!");
            return;
        }

        // Eliminar paredes existentes
        foreach (Transform child in gridParent.transform)
        {
            if (child.gameObject.name.Contains("Wall"))
            {
                Destroy(child.gameObject);
            }
        }

        float gridSizeWorld = gridSize * cellSize;

        // Coordenadas para paredes horizontales (frontal y trasera)
        Vector3 frontWallPosition = new Vector3(gridSizeWorld / 2, 5, 0);
        Vector3 backWallPosition = new Vector3(gridSizeWorld / 2, 5, 20);

        // Coordenadas para paredes verticales (izquierda y derecha)
        Vector3 leftWallPosition = new Vector3(1f, 5f, 10);
        Vector3 rightWallPosition = new Vector3(20f, 5f, 10);

        // Escalas ajustadas
        Vector3 horizontalWallScale = new Vector3(10, 5, 5); // Largo en X
        Vector3 verticalWallScale = new Vector3(10, 5, 5);    // Largo en Z

        // Crear y rotar paredes horizontales
        GameObject frontWall = Instantiate(wallPrefab, frontWallPosition, Quaternion.identity, gridParent.transform);
        frontWall.transform.localScale = horizontalWallScale; // Largo horizontal
        frontWall.transform.rotation = Quaternion.Euler(90, 0, 0); // Sin rotación
        frontWall.name = "Front Wall";

        GameObject backWall = Instantiate(wallPrefab, backWallPosition, Quaternion.identity, gridParent.transform);
        backWall.transform.localScale = horizontalWallScale; // Largo horizontal
        backWall.transform.rotation = Quaternion.Euler(90, 0, 0); // Sin rotación
        backWall.name = "Back Wall";

        // Crear y rotar paredes verticales
        GameObject leftWall = Instantiate(wallPrefab, leftWallPosition, Quaternion.identity, gridParent.transform);
        leftWall.transform.localScale = verticalWallScale; // Largo vertical
        leftWall.transform.rotation = Quaternion.Euler(90, 0, 90); // Girar para alinearse a 
        leftWall.name = "Left Wall";

        GameObject rightWall = Instantiate(wallPrefab, rightWallPosition, Quaternion.identity, gridParent.transform);
        rightWall.transform.localScale = verticalWallScale; // Largo vertical
        rightWall.transform.rotation = Quaternion.Euler(90, 0, 90); // Girar para alinearse a 
        rightWall.name = "Right Wall";
    }
    */

    void GeneratePosts()
    {
        if (postPrefab == null)
        {
            Debug.LogError("Post Prefab is not assigned!");
            return;
        }

        // Eliminar postes existentes
        foreach (Transform child in gridParent.transform)
        {
            if (child.gameObject.name.Contains("Post"))
            {
                Destroy(child.gameObject);
            }
        }

        float gridSizeWorld = gridSize * cellSize;

        // Generar postes en las esquinas
        Vector3[] postPositions = new Vector3[]
        {
        new Vector3(0, postHeight / 2, 0),
        new Vector3(gridSizeWorld, postHeight / 2, 0),
        new Vector3(0, postHeight / 2, gridSizeWorld),
        new Vector3(gridSizeWorld, postHeight / 2, gridSizeWorld)
        };

        for (int i = 0; i < postPositions.Length; i++)
        {
            GameObject post = Instantiate(postPrefab, postPositions[i], Quaternion.identity, gridParent.transform);
            post.transform.localScale = new Vector3(0.5f, postHeight, 0.5f);
            post.name = $"Post_{i + 1}";
        }
    }

    void AddRandomScenery()
    {
        if (sceneryPrefabs == null || sceneryPrefabs.Length == 0)
        {
            Debug.LogWarning("No scenery prefabs assigned.");
            return;
        }

        for (int i = 0; i < numberOfSceneryObjects; i++)
        {
            float x = UnityEngine.Random.Range(0, gridSize * cellSize);
            float z = UnityEngine.Random.Range(0, gridSize * cellSize);
            Vector3 position = new Vector3(x, 0, z);

            GameObject prefab = sceneryPrefabs[UnityEngine.Random.Range(0, sceneryPrefabs.Length)];
            GameObject sceneryObject = Instantiate(prefab, position, Quaternion.identity, gridParent.transform);

            // Rotación aleatoria en el eje Y
            sceneryObject.transform.rotation = Quaternion.Euler(0, UnityEngine.Random.Range(0, 360), 0);

            // Escala aleatoria (opcional)
            float scale = UnityEngine.Random.Range(0.8f, 1.2f);
            sceneryObject.transform.localScale *= scale;

            sceneryObject.name = $"SceneryObject_{i}";
        }
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
        // Actualizaci�n del dron
        UpdateDrones(state.drones);

        // Actualizaci�n de c�maras
        UpdateCameras(state.cameras);

        // Actualizaci�n de ladrones
        UpdateRobbers(state.robbers);

        // Actualizaci�n de personal de seguridad
        UpdateSecurity(state.watchers);

        // Actualizaci�n de zonas de aterrizaje
        UpdateLandingZones(state.landing_stations);
    }

    void UpdateDrones(DroneState[] drones)
    {
        if (drones != null && drones.Length > 0)
        {
            var firstDrone = drones[0];
            Vector3 targetPosition = new Vector3(
                firstDrone.x * cellSize,
                firstDrone.height * cellSize,
                firstDrone.y * cellSize
            );

            if (droneObject != null && (firstDrone.is_flying || firstDrone.height > 0))
            {
                StartCoroutine(SmoothMove(droneObject, targetPosition, 0.5f)); // Movimiento a velocidad 2.5f
            }
        }
    }

    private Dictionary<int, GameObject> cameraObjectsDict = new Dictionary<int, GameObject>();
    private Dictionary<int, GameObject> postObjectsDict = new Dictionary<int, GameObject>();

    void UpdateCameras(CameraState[] cameras)
    {
        HashSet<int> currentCameraIds = new HashSet<int>();
        if (cameras != null)
        {
            foreach (var cam in cameras)
            {
                currentCameraIds.Add(cam.id);
                if (cameraObjectsDict.ContainsKey(cam.id))
                {
                    // Actualizar posición de cámaras y postes existentes
                    GameObject existingCamera = cameraObjectsDict[cam.id];
                    GameObject existingPost = postObjectsDict[cam.id];
                    float cameraHeight = 3.0f;
                    Vector3 position = new Vector3(cam.x * cellSize, 0, cam.y * cellSize);

                    existingCamera.transform.position = new Vector3(position.x, cameraHeight, position.z);
                    existingPost.transform.position = position;
                }
                else
                {
                    // Crear nueva cámara y poste si no existen
                    if (cameraPrefab != null && postPrefab != null)
                    {
                        float cameraHeight = 3.0f;
                        Vector3 position = new Vector3(cam.x * cellSize, 0, cam.y * cellSize);

                        // Crear poste
                        GameObject postObject = Instantiate(postPrefab, position, Quaternion.identity);
                        postObject.name = $"Post_{cam.id}";
                        postObjectsDict[cam.id] = postObject;

                        // Crear cámara
                        GameObject cameraObject = Instantiate(cameraPrefab, new Vector3(position.x, cameraHeight, position.z), Quaternion.identity);
                        cameraObject.name = $"Camera_{cam.id}";
                        cameraObject.transform.SetParent(postObject.transform);

                        // Hacer que la cámara mire hacia el centro
                        if (landingZoneObject != null)
                        {
                            cameraObject.transform.LookAt(landingZoneObject.transform);
                        }

                        // Agregar la cámara y el poste al diccionario
                        cameraObjectsDict[cam.id] = cameraObject;
                    }
                    else
                    {
                        Debug.LogError("Camera Prefab or Post Prefab is not assigned.");
                    }
                }
            }
        }

        // Limpieza: Eliminar cámaras y postes que ya no están en la lista de estados
        var keysToRemove = new List<int>();
        foreach (var cameraId in cameraObjectsDict.Keys)
        {
            if (!currentCameraIds.Contains(cameraId))
            {
                Destroy(cameraObjectsDict[cameraId]);
                Destroy(postObjectsDict[cameraId]);
                keysToRemove.Add(cameraId);
            }
        }
        foreach (var key in keysToRemove)
        {
            cameraObjectsDict.Remove(key);
            postObjectsDict.Remove(key);
        }
    }


    private Dictionary<int, GameObject> robberObjectsDict = new Dictionary<int, GameObject>();

    void UpdateRobbers(RobberState[] robbers)
    {
        HashSet<int> currentRobberIds = new HashSet<int>();

        if (robbers != null)
        {
            foreach (var robber in robbers)
            {
                currentRobberIds.Add(robber.id);

                if (robberObjectsDict.ContainsKey(robber.id))
                {
                    // Actualizar posición del ladrón existente
                    GameObject robberObject = robberObjectsDict[robber.id];
                    if (!robber.is_caught)
                    {
                        Vector3 targetPosition = new Vector3(robber.x * cellSize, 1, robber.y * cellSize);
                        StartCoroutine(SmoothMove(robberObject, targetPosition, 0.5f)); // Mover con Lerp en 0.5 segundos
                    }
                    else
                    {
                        // Si el ladrón ha sido atrapado, eliminar el objeto
                        Destroy(robberObject);
                        robberObjectsDict.Remove(robber.id);
                    }
                }
                else
                {
                    // Crear un nuevo ladrón si no está atrapado
                    if (robberPrefab != null && !robber.is_caught)
                    {
                        GameObject robberObject = Instantiate(
                            robberPrefab,
                            new Vector3(robber.x * cellSize, 1, robber.y * cellSize),
                            Quaternion.identity
                        );
                        robberObjectsDict[robber.id] = robberObject;
                    }
                }
            }
        }

        // Eliminar ladrones que ya no están en el estado
        var keysToRemove = new List<int>();
        foreach (var robberId in robberObjectsDict.Keys)
        {
            if (!currentRobberIds.Contains(robberId))
            {
                Destroy(robberObjectsDict[robberId]);
                keysToRemove.Add(robberId);
            }
        }

        foreach (var key in keysToRemove)
        {
            robberObjectsDict.Remove(key);
        }
    }

    private Dictionary<int, GameObject> securityObjectsDict = new Dictionary<int, GameObject>();
    void UpdateSecurity(WatcherState[] watchers)
    {
        HashSet<int> currentSecurityIds = new HashSet<int>();

        if (watchers != null)
        {
            foreach (var guard in watchers)
            {
                currentSecurityIds.Add(guard.id);

                Vector3 targetPosition = new Vector3(guard.x * cellSize, 1, guard.y * cellSize);

                if (securityObjectsDict.ContainsKey(guard.id))
                {
                    GameObject securityObject = securityObjectsDict[guard.id];
                    StartCoroutine(SmoothMove(securityObject, targetPosition, 0.5f)); // Movimiento a velocidad 2.5f
                }
                else
                {
                    // Crear nuevo policía
                    if (securityPrefab != null)
                    {
                        GameObject securityObject = Instantiate(
                            securityPrefab,
                            targetPosition,
                            Quaternion.identity
                        );
                        securityObjectsDict[guard.id] = securityObject;
                    }
                }
            }
        }

        // Eliminar policías que ya no están en el estado
        var keysToRemove = new List<int>();
        foreach (var securityId in securityObjectsDict.Keys)
        {
            if (!currentSecurityIds.Contains(securityId))
            {
                Destroy(securityObjectsDict[securityId]);
                keysToRemove.Add(securityId);
            }
        }

        foreach (var key in keysToRemove)
        {
            securityObjectsDict.Remove(key);
        }
    }



    void UpdateLandingZones(int[][] landingStations)
    {
        if (landingZoneObject == null && landingStations != null && landingStations.Length > 0)
        {
            if (landingZonePrefab != null)
            {
                var station = landingStations[0];
                landingZoneObject = Instantiate(
                    landingZonePrefab,
                    new Vector3(station[0] * cellSize, 0, station[1] * cellSize),
                    Quaternion.identity
                );
                landingZoneObject.name = "LandingZone";
                Debug.Log($"Landing Zone Spawned at ({station[0]}, {station[1]}).");
            }
            else
            {
                Debug.LogError("Landing Zone Prefab is not assigned.");
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

    IEnumerator SmoothMove(GameObject obj, Vector3 targetPosition, float duration)
    {
        Vector3 startPosition = obj.transform.position;
        float elapsedTime = 0f;

        while (elapsedTime < duration)
        {
            obj.transform.position = Vector3.Lerp(startPosition, targetPosition, elapsedTime / duration);
            elapsedTime += Time.deltaTime;
            yield return null;
        }

        obj.transform.position = targetPosition; // Asegurarse de que llegue exactamente al objetivo
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
