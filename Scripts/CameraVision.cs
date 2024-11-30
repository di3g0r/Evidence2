using UnityEngine;
using System.Collections;
using UnityEngine.Networking;
using System.Linq;

public class CameraVision : MonoBehaviour
{
    [Header("Referencias")]
    public Camera cameraComponent;
    private RenderTexture renderTexture;
    public GameObject landingZonePrefab;
    private Texture2D screenShot;

    [Header("Configuraciones de Visión")]
    public int captureWidth = 256;
    public int captureHeight = 256;
    public float captureInterval = 0.5f;
    public string serverEndpoint = "http://127.0.0.1:5000/camera-vision";

    [Header("Identificación de Cámara")]
    public string cameraID = "1";

    void Awake()
    {
        Debug.Log("CameraVision Awake() llamado");

        cameraComponent = GetComponent<Camera>();
        if (cameraComponent == null)
        {
            cameraComponent = GetComponentInChildren<Camera>();
        }

        if (cameraComponent == null)
        {
            Debug.LogError("No se pudo encontrar una Cámara!");
            return;
        }

        cameraComponent.enabled = true;
        cameraComponent.fieldOfView = 60;
        cameraComponent.nearClipPlane = 0.1f;
        cameraComponent.farClipPlane = 100f;

        landingZonePrefab = GameObject.Find("LandingZone");

        if (landingZonePrefab == null)
        {
            Debug.LogWarning("No se encontró el LandingZone al inicio. Intentando localizar más tarde.");
        }

        renderTexture = new RenderTexture(captureWidth, captureHeight, 24);
        cameraComponent.targetTexture = renderTexture;

        screenShot = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);

        Debug.Log("CameraVision inicializado exitosamente");
        StartCoroutine(CaptureAndSendLoop());
    }

    void Update()
    {
        // Actualizar referencia a la LandingZone si no existe
        if (landingZonePrefab == null)
        {
            landingZonePrefab = GameObject.Find("LandingZone");
            if (landingZonePrefab != null)
            {
                Debug.Log("LandingZone encontrado dinámicamente.");
            }
        }

        // Apuntar hacia la LandingZone si existe
        if (landingZonePrefab != null)
        {
            cameraComponent.transform.LookAt(landingZonePrefab.transform);
        }
    }

    private string GetCameraID()
    {
        string objectName = gameObject.name;
        string[] parts = objectName.Split('_');
        if (parts.Length > 1 && int.TryParse(parts[1], out int id))
        {
            return id.ToString();
        }
        return "Unknown";
    }

    IEnumerator CaptureAndSendLoop()
    {
        int frameCount = 0;
        Debug.Log("CaptureAndSendLoop iniciado");

        while (true)
        {
            yield return new WaitForSeconds(captureInterval);

            if (cameraComponent == null || renderTexture == null)
            {
                Debug.LogError("¡Cámara o RenderTexture es nulo!");
                yield break;
            }

            // Captura de imagen
            RenderTexture.active = renderTexture; // Establecer RenderTexture activo
            cameraComponent.Render();            // Forzar el renderizado de la cámara

            screenShot.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);
            screenShot.Apply();
            RenderTexture.active = null;         // Restablecer contexto de RenderTexture

            // Verificación de datos de imagen
            byte[] bytes = screenShot.GetRawTextureData();
            Debug.Log($"Frame {frameCount} - Captura realizada, tamaño: {bytes.Length} bytes");

            // Enviar datos al servidor
            WWWForm form = new WWWForm();
            form.AddBinaryData("image", bytes, $"camera_{GetCameraID()}.raw", "application/octet-stream");
            form.AddField("camera_id", GetCameraID());

            using (UnityWebRequest www = UnityWebRequest.Post(serverEndpoint, form))
            {
                yield return www.SendWebRequest();
                Debug.Log($"Frame {frameCount} - Solicitud completada con resultado: {www.result}");

                if (www.result == UnityWebRequest.Result.Success)
                {
                    Debug.Log($"Frame {frameCount} - Respuesta del servidor: {www.downloadHandler.text}");
                }
                else
                {
                    Debug.LogError($"Frame {frameCount} - Error: {www.error}");
                }
            }

            frameCount++;
        }
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Debug.Log("RenderTexture liberado");
        }
    }
}
