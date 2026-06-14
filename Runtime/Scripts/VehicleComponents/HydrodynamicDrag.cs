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
        public float AreaRight = 0f;
        public float CdRight = 0f;
        public float AreaUp = 0f;
        public float CdUp = 0f;

        [Header("Added Mass Parameters")]
        public float AMx = 0f;
        public float AMy = 0f;
        public float AMz = 0f;

        [Header("Roll Dampening")]
        public float KRoll = 0.3f; // This is made up and not based in reality

        private readonly float waterDensity = 997;
        private Vector3 prev_vel = Vector3.zero;

        void FixedUpdate()
        {
            if (Physics.simulationMode == SimulationMode.FixedUpdate) DoUpdate();
        }

        public void DoUpdate()
        {
            Vector3 vel = Body.linearVelocity;
            Vector3 fwd = transform.right;
            Vector3 right = transform.forward;
            Vector3 up = transform.up;

            float vFwd = Vector3.Dot(vel, fwd);
            float vRight = Vector3.Dot(vel, right);
            float vUp = Vector3.Dot(vel, up);

            // Mathf.Abs to remember the direction and oppose it.
            float fDragFwd = -0.5f * waterDensity * CdFwd * AreaFwd * vFwd * Mathf.Abs(vFwd);
            float fDragRight = -0.5f * waterDensity * CdRight * AreaRight * vRight * Mathf.Abs(vRight);
            float fDragUp = -0.5f * waterDensity * CdUp * AreaUp * vUp * Mathf.Abs(vUp);

            Vector3 dragForce = fDragFwd * fwd + fDragRight * right + fDragUp * up;

            float dt = Time.fixedDeltaTime;
            Vector3 accel = (vel - prev_vel) / dt;
            float aFwd = Vector3.Dot(accel, fwd);
            float aRight = Vector3.Dot(accel, right);
            float aUp = Vector3.Dot(accel, up);
            // float Xudot = 0.1f * Body.mass;
            // float Yvdot = 0.5f * Body.mass;
            // float Zwdot = Yvdot;
            // Vector3 aMForce = -(Xudot * aFwd * fwd + Yvdot * aRight * right + Zwdot * aUp * up);
            Vector3 aMForce = -(AMx * aFwd * fwd + AMy * aRight * right + AMz * aUp * up);

            Vector3 totalForce = dragForce + aMForce;
            // Debug.Log($"fwd:{transform.forward}, right:{transform.right}, up:{transform.up}");
            // Debug.Log($"{gameObject.name} velocity: {vel}, dragForce: {totalForce}, local vel: {vFwd}, {vRight}, {vUp}");
            Body.AddForce(totalForce);
            prev_vel = vel;

            Vector3 angVel = Body.angularVelocity;
            float wRoll = Vector3.Dot(angVel, fwd);
            Body.AddTorque(-KRoll * wRoll * fwd);

        }

        public void Reset()
        {
            prev_vel = Vector3.zero;
        }
    }
}
