using UnityEngine;
using System.Collections;
using UnityEngine.Networking;
using System.IO;

public class DroneVision : MonoBehaviour
{
    [Header("References")]
    public Camera droneCamera; // Will be assigned in prefab
    private RenderTexture renderTexture;
    private Texture2D screenShot;
    private DroneSimulationVisualizer visualizer;
    
    [Header("Vision Settings")]
    public int captureWidth = 256;
    public int captureHeight = 256;
    public float captureInterval = 0.5f;
    public string serverEndpoint = "http://127.0.0.1:5000/process-vision";

    void Awake()
    {
        Debug.Log("DroneVision Awake() called");
        
        // Find the visualizer
        visualizer = FindFirstObjectByType<DroneSimulationVisualizer>();
        if (visualizer == null)
        {
            Debug.LogError("Could not find DroneSimulationVisualizer!");
            return;
        }
        
        // Find camera in children
        droneCamera = GetComponentInChildren<Camera>();
        if (droneCamera == null)
        {
            Debug.LogError("Could not find Camera in children!");
            return;
        }
        
        // Setup camera
        droneCamera.enabled = true;
        droneCamera.fieldOfView = 60;
        droneCamera.nearClipPlane = 0.1f;
        droneCamera.farClipPlane = 100f;
        
        // Create render texture
        renderTexture = new RenderTexture(captureWidth, captureHeight, 24);
        droneCamera.targetTexture = renderTexture;
        
        // Create screenshot texture
        screenShot = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
        
        Debug.Log("DroneVision initialized successfully");
        StartCoroutine(CaptureAndSendLoop());
    }

    IEnumerator CaptureAndSendLoop()
    {
        int frameCount = 0;
        Debug.Log("CaptureAndSendLoop started");
        
        while (true)
        {
            yield return new WaitForSeconds(captureInterval);
            
            if (droneCamera == null || renderTexture == null)
            {
                Debug.LogError("Camera or RenderTexture is null!");
                yield break;
            }
            
            Debug.Log($"Frame {frameCount} - About to capture");
            droneCamera.Render();
            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);
            screenShot.Apply();
            
            byte[] bytes = screenShot.GetRawTextureData();
            Debug.Log($"Frame {frameCount} - Captured image size: {bytes.Length} bytes");
            
            WWWForm form = new WWWForm();
            form.AddBinaryData("image", bytes, "drone_view.raw", "application/octet-stream");
            
            using (UnityWebRequest www = UnityWebRequest.Post(serverEndpoint, form))
            {
                yield return www.SendWebRequest();
                Debug.Log($"Frame {frameCount} - Request completed with result: {www.result}");
                
                if (www.result == UnityWebRequest.Result.Success)
                {
                    Debug.Log($"Frame {frameCount} - Server response: {www.downloadHandler.text}");
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
            Debug.Log("RenderTexture released");
        }
    }
}
