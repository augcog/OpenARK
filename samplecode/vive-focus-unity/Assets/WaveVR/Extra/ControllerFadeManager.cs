using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(FadeManger))]
public class ControllerFadeManager : MonoBehaviour {
    private FadeManger fadeManager;

    void Start () {
        fadeManager = GetComponent<FadeManger>();

        MeshRenderer[] renderers = GetComponentsInChildren<MeshRenderer>();
        fadeManager.Materials = new List<Material>();

        foreach (var renderer in renderers)
        {
            if (renderer == null)
                continue;

            Material material = renderer.material;
            if (material != null && material.shader.name == "WaveVR/UnlitControllerShader")
            {
                if (material.HasProperty("_FadeAlpha") && !fadeManager.Materials.Contains(material))
                    fadeManager.Materials.Add(material);
            }
        }
    }

    private float AngleHide = 15; // 0-90

    void Update () {
        // Fade out when pitch angle is high.
        // angle here is in degree
        if (fadeManager != null)
        {
            float angle = Mathf.Acos(Vector3.Dot(transform.forward, Vector3.up)) * Mathf.Rad2Deg;
            if (angle < AngleHide)
            {
                fadeManager.Fade(true);
            }
            else
            {
                fadeManager.Fade(false);
            }
        }
    }
}
