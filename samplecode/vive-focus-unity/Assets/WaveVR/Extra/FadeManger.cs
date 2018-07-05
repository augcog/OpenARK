using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// The target materials shoud have "_FadeAlpha" property.
public class FadeManger : MonoBehaviour {
    public float FadeTime = 0.2f;  // 2ms
    public List<Material> Materials;

    private float InternalFadeTime = 0.2f;  // Avoid change when fading.

    // Default is show and fade out.
    private bool isInTransition = false;
    private bool IsFadeOut = false;
    private float alpha = 1;

    public void Fade(bool fadeOut)
    {
        if (isInTransition)
        {
            // Nothing to do
            if (IsFadeOut == fadeOut)
                return;

            // Change fade direction
            IsFadeOut = fadeOut;
            alpha = 1 - alpha;
        }
        else
        {
            // Nothing to do
            if (IsFadeOut == fadeOut)
                return;

            isInTransition = true;
            IsFadeOut = fadeOut;

            InternalFadeTime = FadeTime;
            alpha = IsFadeOut ? 1 : 0;
        }
    }
	
	// Update is called once per frame
	void Update()
    {
        //if (Input.GetKeyDown(KeyCode.G))
        //    Fade(true);
        //if (Input.GetKeyDown(KeyCode.H))
        //    Fade(false);

        if (!isInTransition)
            return;

        if (Materials == null || Materials.Count == 0)
            return;

        if (IsFadeOut)
        {
            alpha -= Time.deltaTime * (1 / InternalFadeTime);
            if (alpha < 0)
            {
                alpha = 0;
                isInTransition = false;
            }
        }
        else
        {
            alpha += Time.deltaTime * (1 / InternalFadeTime);
            if (alpha > 1)
            {
                isInTransition = false;
                alpha = 1;
            }
        }

        foreach (var material in Materials)
        {
            material.SetFloat("_FadeAlpha", alpha);
        }
    }
}
