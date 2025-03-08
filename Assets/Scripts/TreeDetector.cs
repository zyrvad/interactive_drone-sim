using System;
using System.Collections.Generic;
using UnityEngine;

public class TreeDetector : MonoBehaviour
{
    [SerializeField] private float cameraDistance = 40f;
    [SerializeField] private bool outputCameraData = false;
    private Transform treeParent;
    private Camera cam;

    // Store bounding boxes for rendering
    private List<Rect> boundingBoxes = new List<Rect>();

    void Start()
    {
        cam = GetComponent<Camera>();
    }

    void Update()
    {
        if (!treeParent) treeParent = GameObject.Find("Trees").transform;
        String output = "";

        // Clear bounding boxes for this frame
        boundingBoxes.Clear();

        foreach (Transform child in treeParent)
        {
            if (Vector3.Distance(child.position, transform.position) > cameraDistance) continue;

            Vector4 xywh = CheckTreeVisibilityAndBoundingBox(child.gameObject);

            if (xywh != Vector4.zero)
            {
                // Add bounding box to the list
                boundingBoxes.Add(new Rect(xywh.x, Screen.height - xywh.y - xywh.w, xywh.z, xywh.w)); // Convert Y-coordinate for GUI
                
                output += $"{xywh.x}, {xywh.y}, {xywh.z}, {xywh.w}\n";
            }
        }

        if (outputCameraData) Debug.Log(output);
    }

    public Vector4 CheckTreeVisibilityAndBoundingBox(GameObject tree)
    {
        CapsuleCollider capsuleCollider = tree.GetComponent<CapsuleCollider>();
        if (capsuleCollider == null) { Debug.Log("no collider"); return Vector4.zero; }

        Vector3[,] corners = new Vector3[2, 4];

        corners[0, 0] = cam.WorldToScreenPoint(new Vector3(capsuleCollider.bounds.min.x, capsuleCollider.bounds.max.y, capsuleCollider.bounds.min.z)); // top left back
        corners[0, 1] = cam.WorldToScreenPoint(new Vector3(capsuleCollider.bounds.min.x, capsuleCollider.bounds.max.y, capsuleCollider.bounds.max.z)); // top right back
        corners[0, 2] = cam.WorldToScreenPoint(new Vector3(capsuleCollider.bounds.max.x, capsuleCollider.bounds.max.y, capsuleCollider.bounds.max.z)); // top right front
        corners[0, 3] = cam.WorldToScreenPoint(capsuleCollider.bounds.max); // top left front

        corners[1, 0] = cam.WorldToScreenPoint(capsuleCollider.bounds.min); // bottom left back
        corners[1, 1] = cam.WorldToScreenPoint(new Vector3(capsuleCollider.bounds.min.x, capsuleCollider.bounds.min.y, capsuleCollider.bounds.max.z)); // bottom right back
        corners[1, 2] = cam.WorldToScreenPoint(new Vector3(capsuleCollider.bounds.max.x, capsuleCollider.bounds.min.y, capsuleCollider.bounds.max.z)); // bottom right front
        corners[1, 3] = cam.WorldToScreenPoint(new Vector3(capsuleCollider.bounds.max.x, capsuleCollider.bounds.min.y, capsuleCollider.bounds.min.z)); // bottom left front

        bool inFrame = IsAnyCornerInFrame(cam, corners);
        if (!inFrame) return Vector4.zero;

        float minX = float.MaxValue, maxX = float.MinValue;
        float minY = float.MaxValue, maxY = float.MinValue;

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                Vector3 screenPoint = corners[i, j];
                if (screenPoint.z > 0)
                {
                    minX = Mathf.Min(minX, screenPoint.x);
                    maxX = Mathf.Max(maxX, screenPoint.x);
                    minY = Mathf.Min(minY, screenPoint.y);
                    maxY = Mathf.Max(maxY, screenPoint.y);
                }
            }
        }

        float x = Mathf.Clamp(minX, 0, Screen.width);
        float y = Mathf.Clamp(minY, 0, Screen.height);
        float w = maxX - minX;
        float h = maxY - minY;

        return new Vector4(x, y, w, h);
    }

    bool IsAnyCornerInFrame(Camera cam, Vector3[,] corners)
    {
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                Vector3 screenPoint = corners[i, j];
                if (screenPoint.z > 0 &&
                    screenPoint.x >= 0 && screenPoint.x <= Screen.width &&
                    screenPoint.y >= 0 && screenPoint.y <= Screen.height)
                {
                    return true;
                }
            }
        }
        return false;
    }

    public List<Vector4> GetTreeBoundingBoxes()
    {
        List<Vector4> boundingBoxes = new List<Vector4>();
        if(treeParent!=null)
        {
             foreach (Transform child in treeParent)
            {
                if (Vector3.Distance(child.position, transform.position) > cameraDistance) continue;

                Vector4 bbox = CheckTreeVisibilityAndBoundingBox(child.gameObject);
                if (bbox != Vector4.zero)
                {
                    boundingBoxes.Add(bbox);
                }
            }
        }
        return boundingBoxes;
    }

    /* void OnGUI()
    {
        // Draw bounding boxes
        foreach (var box in boundingBoxes)
        {
            GUI.color = Color.red;
            GUI.DrawTexture(box, Texture2D.whiteTexture, ScaleMode.StretchToFill);
        }
    } */
}
