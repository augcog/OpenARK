using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/** Controller.cs: Add this script to an empty game object and follow the instructions below. */
public class Controller : MonoBehaviour {
    public float scale = 100.0f; // the scaling factor between real world and Unity distances. You can leave this at 100.
    public GameObject palm; // create and drag a palm object to this field in the unity editor
    public GameObject[] fingers; // create and drag five finger objects to this field in the unity editor
    
    private OpenARK.Detector detector;

    void Start () {
        detector = new OpenARK.Detector(true);
    }

    void Update () {
        detector.update();
        List<OpenARK.Hand> hands = detector.getHands();
        if (hands.Count > 0)
        {
            OpenARK.Hand hand = hands[0];
            palm.transform.position = hand.center * scale;
            for (int i = 0; i < Mathf.Min(hand.fingers.Length, fingers.Length); ++i)
            {
                fingers[i].SetActive(true);
                fingers[i].transform.position = hand.fingers[i] * scale;
            }

            for (int i = hand.fingers.Length; i < fingers.Length; ++i)
            {
                fingers[i].SetActive(false);
            }
        }
    }
}