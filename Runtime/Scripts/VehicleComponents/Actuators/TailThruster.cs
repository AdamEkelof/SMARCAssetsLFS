using UnityEngine;
using Force; // MixedBody is in the Force namespace

namespace VehicleComponents.Actuators
{
    public enum TailSecondaryDirection
    {
        ZForward,
        YForward
    }
    public class TailThrust : LinkAttachment
    {
        [Header("Fitted force parameters - X (forward)")]
        public float C_x = 2.559f;
        public float A1_x = -1.321f;
        public float A2_x = -3.476f;
        public float p1_x = 5.513f;
        public float p2_x = 1.405f;

        [Header("Fitted force parameters - Y (lateral)")]
        public float A_y = 8.772f;
        public float p_y = -0.718f;

        [Header("Tail beat frequency")]
        public float oscillationFrequency = 0.75f;   // Hz

        // Internal phase of the tail
        private float phase = 0f;
        private float forceX = 0f;
        private float forceY = 0f;
        public float ForceX => forceX;
        public float ForceY => forceY;

        new void Awake()
        {
            base.Awake();
        }

        new void FixedUpdate()
        {
            base.FixedUpdate();
            if (Physics.simulationMode == SimulationMode.FixedUpdate) DoUpdate();
        }

        public void DoUpdate()
        {
            phase += oscillationFrequency * 2f * Mathf.PI * Time.fixedDeltaTime;

            // Y force - sweep, single sine at f0
            forceY = A_y * Mathf.Sin(phase + p_y);

            // X force - thrust, DC + fundamental asymmetry + second harmonic
            forceX = C_x
                         + A1_x * Mathf.Sin(1f * phase + p1_x)
                         + A2_x * Mathf.Sin(2f * phase + p2_x);

            Vector3 thrustX = forceX * parentMixedBody.transform.right;
            Vector3 thrustY = forceY * parentMixedBody.transform.forward;

            parentMixedBody.AddForceAtPosition(
                thrustX + thrustY,
                parentMixedBody.transform.position,
                ForceMode.Force);
        }

        public void Reset()
        {
            phase = 0f;
        }
    }
}
