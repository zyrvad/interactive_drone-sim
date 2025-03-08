using UnityEngine;

public class BoundingBoxDrawer : MonoBehaviour
{
    public Transform target;
    public Camera agentCamera;

    void OnGUI()
    {
        Renderer targetRenderer = target.GetComponent<Renderer>();
        Bounds targetBounds = targetRenderer.bounds;
        Vector3 screenMin = agentCamera.WorldToScreenPoint(targetBounds.min);
        Vector3 screenMax = agentCamera.WorldToScreenPoint(targetBounds.max);
        Vector3 screenCenter = (screenMin + screenMax) / 2f;

        bool targetVisible = screenMax.z > 0 && screenMin.x > 0 && screenMin.y > 0 && screenMin.x < Screen.width && screenMin.y < Screen.height;

        if (targetVisible)
        {
            // Draw the target's bounding box
            float width = screenMax.x - screenMin.x;
            float height = screenMax.y - screenMin.y;
            GUI.color = Color.green;
            GUI.DrawTexture(new Rect(screenMin.x, Screen.height - screenMax.y, width, height), Texture2D.whiteTexture, ScaleMode.StretchToFill);

            // Draw a dot at the center of the target
            GUI.color = Color.red;
            GUI.DrawTexture(new Rect(screenCenter.x - 5, Screen.height - screenCenter.y - 5, 10, 10), Texture2D.whiteTexture);
        }
        else
        {
            // Draw a warning message if the target is not visible
            GUI.color = Color.yellow;
            GUI.Label(new Rect(10, 10, 300, 20), "Target is not visible");
        }
    }
}
