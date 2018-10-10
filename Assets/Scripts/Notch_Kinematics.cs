using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Notch_Kinematics : MonoBehaviour {

    // Tube Parameters in mm
    float rout = 0.58f;         // Outer Radius
    float rin = 0.43f;          // Inner Radius
    float g = 0.97f;
    float a = 3f;               // Base cylinder length
    float b = 3f;               // End effector cylinder length


    // Notch Parameters in mm
    float h = 0.51f;            // Notch Thickness
    float c = 0.51f;            // Distance between 2 notches

    // Max values
    float rhoMin = 1.42f;       // mm
    float tauMax = 10.4f;       // %
    float Ftendon = 5f;         // Newtons 
    float thetaMax = 138.6f;    // degrees
    
    
    // Notch Variables
    int n = 5;                  // No. of notches
    float k;                    // Arc curvature 
    float s;                    // Arc length

    // Inputs
    float delL = 0.3f;         // Tendon Displacement in mm
    float alp = 0f;          // Base rotation in radians

    // Used for creating and storing required components for the kinematics of notch
    List<Matrix4x4> Transforms;
    List<GameObject> empObjs;
    List<GameObject> cylObjs;
    GameObject cylPrefab;


    // Use this for initialization
    void Start ()
    {
        cylPrefab = (GameObject)Resources.Load("cylinder");
        Transforms = new List<Matrix4x4>();
        empObjs = new List<GameObject>(2 * (n + 1));
        cylObjs = new List<GameObject>(2 * (n + 1));

        generateNotchParameters();
        generateCylinders();
        notchKinematics();
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    // Generates values of k and s given the parameters
    void generateNotchParameters()
    {
        float phiOut = 2 * Mathf.Acos((g - rout) / rout);
        float phiIn = 2 * Mathf.Acos((g - rout) / rin);
        float Aout = Mathf.Pow(rout, 2) * (phiOut - Mathf.Sin(phiOut)) / 2;
        float Ain = Mathf.Pow(rin, 2) * (phiIn - Mathf.Sin(phiIn)) / 2;
        float yout = 4 * rout * Mathf.Pow(Mathf.Sin(phiOut / 2), 3) / (3 * (phiOut - Mathf.Sin(phiOut)));
        float yin = 4 * rin * Mathf.Pow(Mathf.Sin(phiIn / 2), 3) / (3 * (phiIn - Mathf.Sin(phiIn)));
        float y = (yout * Aout - yin * Ain) / (Aout - Ain);
        k = delL / (h * (rin + y) - delL * y);
        s = h / (1 + y * k);
    }

    // Generates the cylinders for the stick model
    void generateCylinders()
    {
        foreach (GameObject cylinder in cylObjs)
            Destroy(cylinder);
        foreach (GameObject empty in empObjs)
            Destroy(empty);
        cylObjs.Clear();
        empObjs.Clear();

        cylObjs.Add(Instantiate(cylPrefab, gameObject.transform));
        cylObjs[0].transform.localPosition = new Vector3(0, 0, a / 2);
        cylObjs[0].transform.localScale = new Vector3(1.16f, 1.5f, 1.16f);

        for (int i = 0; i < 2 * (n + 1); ++i)
        {
            empObjs.Add(new GameObject());
            empObjs[i].transform.SetParent(gameObject.transform);
            if (i < 2 * n + 1)
            {
                cylObjs.Add(Instantiate(cylPrefab, empObjs[i].transform));
                if (i == 2 * n)
                {
                    cylObjs[i + 1].transform.localPosition = new Vector3(0, 0, (b - c) / 2);
                    cylObjs[i + 1].transform.localScale = new Vector3(1.16f, 1.245f, 1.16f);
                }
                else
                {
                    if (i % 2 == 0)
                        cylObjs[i + 1].transform.localPosition = new Vector3(0, 0, h / 2);
                    else
                        cylObjs[i + 1].transform.localPosition = new Vector3(0, 0, c / 2);
                }
            }
        }
    }

    // Positions the cylinders based on kinmatics of the notch
    void notchKinematics()
    {
        Matrix4x4 Tbase = Matrix4x4.identity;
        Tbase[0, 0] = -Mathf.Cos(alp);
        Tbase[0, 1] = -Mathf.Sin(alp);
        Tbase[1, 0] = -Mathf.Sin(alp);
        Tbase[1, 1] = Mathf.Cos(alp);

        Matrix4x4 Tstart = Matrix4x4.identity;
        Tstart[0, 0] = -1;
        Tstart[2, 3] = a;
        Tstart = Tbase * Tstart;

        Matrix4x4 Tnotch = Matrix4x4.identity;
        Tnotch[0, 0] = -1;
        Tnotch[1, 1] = Mathf.Cos(k * s);
        Tnotch[1, 2] = -Mathf.Sin(k * s);
        Tnotch[1, 3] = (Mathf.Cos(k * s) - 1) / k;
        Tnotch[2, 1] = Mathf.Sin(k * s);
        Tnotch[2, 2] = Mathf.Cos(k * s);
        Tnotch[2, 3] = Mathf.Sin(k * s) / k;

        Matrix4x4 Tmid = Matrix4x4.identity;
        Tmid[0, 0] = -1;
        Tmid[2, 3] = c;

        Matrix4x4 Tend = Matrix4x4.identity;
        Tend[0, 0] = -1;
        Tend[2, 3] = b - c;

        for (int i = 0; i < 2 * (n + 1); ++i)
        {
            if (i == 0)
                Transforms.Add(Tstart);
            else if (i == 2 * n + 1)
                Transforms.Add(Transforms[i - 1] * Tend);
            else if (i % 2 != 0)
                Transforms.Add(Transforms[i - 1] * Tnotch);
            else
                Transforms.Add(Transforms[i - 1] * Tmid);

            empObjs[i].transform.localPosition = Transforms[i].GetColumn(3);
            empObjs[i].transform.localRotation = Quaternion.LookRotation(Transforms[i].GetColumn(2), Transforms[i].GetColumn(1));
        }
    }
}