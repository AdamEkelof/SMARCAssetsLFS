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
        // private Vector3 prevVel = Vector3.zero; // Assumes the robot is stationary at start

        private void AddForces()
        {
            var vel  = Body.linearVelocity;
            var fwd  = transform.right;
            var left = transform.forward;
            var up   = transform.up;

            var velF = Vector3.Dot(vel, fwd);
            var velL = Vector3.Dot(vel, left);
            var velU = Vector3.Dot(vel, up);

            var dragForce = -.5f * waterDensity * (
                CdFwd  * AreaFwd  * velF * Mathf.Abs(velF) * fwd +
                CdLeft * AreaLeft * velL * Mathf.Abs(velL) * left +
                CdUp   * AreaUp   * velU * Mathf.Abs(velU) * up
            );
            Body.AddForce(dragForce);

            // INFO: This must be run last since the forces need to be added.
            var forces = Body.GetAccumulatedForce(); // If run first forces = 0
            // var g = Body.mass * Physics.gravity;
            // forces += g;

            var forceF = Vector3.Dot(forces, fwd);
            var forceL = Vector3.Dot(forces, left);
            var forceU = Vector3.Dot(forces, up);

            var m = Body.mass;
            var addedMassForce = -(
                AMx / (m + AMx) * forceF * fwd +
                AMy / (m + AMy) * forceL * left +
                AMz / (m + AMz) * forceU * up
            );
            Body.AddForce(addedMassForce);
        }

        private void AddTorque()
        {
            var omega = Body.angularVelocity;
            var fwd   = transform.right;

            var rollRate = Vector3.Dot(omega, fwd);
            Body.AddTorque(-KRoll * rollRate * fwd);
        }

        public void DoUpdate(Vector3 disturbanceForce)
        {
            Body.AddForce(disturbanceForce);
            AddForces();
            AddTorque();
        }

        public void Reset()
        {
        }
    }
}
