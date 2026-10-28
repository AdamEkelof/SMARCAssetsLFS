using UnityEngine;
using Force;

namespace VehicleComponents
{
    public class HydrodynamicDrag : MonoBehaviour
    {
        [Header("Connected Body")]
        public ArticulationBody Body;

        [Header("Drag Parameters")]
        public float AreaFwd = 0f;
        public float CdFwd = 0f;
        public float AreaLeft = 0f;
        public float CdLeft = 0f;
        public float AreaUp = 0f;
        public float CdUp = 0f;

        [Header("Added Mass Parameters")]
        public float AMx = 0f;
        public float AMy = 0f;
        public float AMz = 0f;

        [Header("Roll Dampening")]
        public float KRoll = 0.3f; // This is made up and not based in reality

        private readonly float waterDensity = 997;
        private Vector3 prevVel = Vector3.zero; // Assumes the robot is stationary at start

        void FixedUpdate()
        {
            if (Physics.simulationMode == SimulationMode.FixedUpdate) DoUpdate(Vector3.zero);
        }

        private Vector3 GetDragForce(Vector3 vel, Vector3 fwd, Vector3 left, Vector3 up)
        {
            float velFwd = Vector3.Dot(vel, fwd);
            float velLeft = Vector3.Dot(vel, left);
            float velUp = Vector3.Dot(vel, up);

            // Mathf.Abs to remember the direction and oppose it.
            float dragFwd = -0.5f * waterDensity * CdFwd * AreaFwd * velFwd * Mathf.Abs(velFwd);
            float dragLeft = -0.5f * waterDensity * CdLeft * AreaLeft * velLeft * Mathf.Abs(velLeft);
            float dragUp = -0.5f * waterDensity * CdUp * AreaUp * velUp * Mathf.Abs(velUp);

            Vector3 dragForce =
                dragFwd * fwd +
                dragLeft * left +
                dragUp * up;

            return dragForce;
        }
        private Vector3 GetAddedMassForce(float dt, Vector3 vel, Vector3 omega, Vector3 fwd, Vector3 left, Vector3 up)
        {
            Vector3 accelWorld = (vel - prevVel) / dt;
            Vector3 accelBody = accelWorld - Vector3.Cross(omega, vel);

            float accelFwd = Vector3.Dot(accelBody, fwd);
            float accelLeft = Vector3.Dot(accelBody, left);
            float accelUp = Vector3.Dot(accelBody, up);

            Vector3 addedMassForce =
                -(AMx * accelFwd * fwd +
                AMy * accelLeft * left +
                AMz * accelUp * up);

            return addedMassForce;
        }

        public void DoUpdate(Vector3 disturbanceForce)
        {
            float dt = Time.fixedDeltaTime;
            Vector3 vel = Body.linearVelocity;
            Vector3 omega = Body.angularVelocity;

            // X is forward, Y is left, Z is up.
            Vector3 fwd = transform.right;
            Vector3 left = transform.forward;
            Vector3 up = transform.up;

            Vector3 dragForce = GetDragForce(vel, fwd, left, up);
            Vector3 addedMassForce = GetAddedMassForce(dt, vel, omega, fwd, left, up);

            Body.AddForce(dragForce + addedMassForce + disturbanceForce);
            prevVel = vel;

            float rollRate = Vector3.Dot(omega, fwd);
            Body.AddTorque(-KRoll * rollRate * fwd);
        }

        public void Reset()
        {
            prevVel = Vector3.zero;
        }
    }
}
