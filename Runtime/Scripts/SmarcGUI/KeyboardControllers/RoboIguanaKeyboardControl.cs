using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Hinge = VehicleComponents.Actuators.Hinge;

namespace SmarcGUI.KeyboardControllers
{
    public class RoboIguanaIKController : KeyboardControllerBase
    {
        // =========================================================
        // ACTUATORS
        // =========================================================

        [Header("Hip yaw actuators")]
        public Hinge hyFL, hyFR, hyRL, hyRR;

        [Header("Hip pitch actuators")]
        public Hinge hpFL, hpFR, hpRL, hpRR;

        [Header("Knee actuators")]
        public Hinge kFL, kFR, kRL, kRR;

        // =========================================================
        // PHYSICAL JOINTS
        // =========================================================

        [Header("Hip yaw links")]
        public ArticulationBody hyFL_Link;
        public ArticulationBody hyFR_Link;
        public ArticulationBody hyRL_Link;
        public ArticulationBody hyRR_Link;

        [Header("Hip pitch links")]
        public ArticulationBody hpFL_Link;
        public ArticulationBody hpFR_Link;
        public ArticulationBody hpRL_Link;
        public ArticulationBody hpRR_Link;

        [Header("Knee links")]
        public ArticulationBody kFL_Link;
        public ArticulationBody kFR_Link;
        public ArticulationBody kRL_Link;
        public ArticulationBody kRR_Link;

        // =========================================================
        // SPINE
        // =========================================================

        [Header("Spine actuator")]
        public Hinge spine;

        [Header("Spine link")]
        public ArticulationBody spine_Link;

        [Header("Spine control")]
        public bool activeSpineControl = true;

        [Range(0f, 20f)]
        public float spineAmplitudeDeg = 10f;

        private bool spineDriveDisabled = false;

        // =========================================================
        // TIMING
        // =========================================================

        [Header("Gait timing")]
        public float frequencyHz = 0.5f;
        public float startDelay = 3f;

        float time;

        // =========================================================
        // LEG GEOMETRY
        // =========================================================

        [Header("Leg geometry (meters)")]
        public float a = 0.116f;
        public float b = 0.07f;
        public float c = 0.172f;
        public float d = 0.19f;

        // =========================================================
        // FOOT TRAJECTORY
        // =========================================================

        [Header("Foot trajectory")]
        public Vector3 centerRight = new Vector3(-0.07f, -0.2f, -0.2f);
        public Vector3 centerLeft = new Vector3(0.07f, -0.2f, -0.2f);

        public float Rx = 0.1f;
        public float Ry = 0.03f;

        // =========================================================
        // INITIALISE JOINT
        // =========================================================

        void InitialiseJoint(ArticulationBody ab, Hinge h, float angleRad)
        {
            ab.jointPosition = new ArticulationReducedSpace(angleRad);
            var drive = ab.xDrive;
            drive.target = angleRad;
            ab.xDrive = drive;
            h.SetAngle(angleRad);
        }

        // =========================================================
        // INITIAL POSE
        // =========================================================

        void OnEnable()
        {
            time = 0f;

            float phaseA = 0;
            float phaseB = Mathf.PI;

            SolveAngles(
                phaseA, phaseB,
                out float yawA, out float hipA, out float kneeA,
                out float yawB, out float hipB, out float kneeB
            );

            InitialiseJoint(hyFL_Link, hyFL, Mathf.PI + yawA);
            InitialiseJoint(hyRR_Link, hyRR, -Mathf.PI - yawA);
            InitialiseJoint(hyFR_Link, hyFR, -Mathf.PI - yawB);
            InitialiseJoint(hyRL_Link, hyRL, Mathf.PI + yawB);

            InitialiseJoint(hpFL_Link, hpFL, hipA);
            InitialiseJoint(hpRR_Link, hpRR, hipA);
            InitialiseJoint(hpFR_Link, hpFR, hipB);
            InitialiseJoint(hpRL_Link, hpRL, hipB);

            InitialiseJoint(kFL_Link, kFL, kneeA);
            InitialiseJoint(kRR_Link, kRR, kneeA);
            InitialiseJoint(kFR_Link, kFR, kneeB);
            InitialiseJoint(kRL_Link, kRL, kneeB);

            if (spine_Link != null && spine != null)
            {
                InitialiseJoint(spine_Link, spine, 0.174533f);
            }
        }

        // =========================================================
        // UPDATE
        // =========================================================

        void Update()
        {
            time += Time.deltaTime;

            if (time < startDelay)
                return;

            float gaitTime = time - startDelay;
            float T = 1f / frequencyHz;

            float halfT = T * 0.5f;
            float pause = T;

            float segmentTime = halfT + pause;
            float extendedPeriod = 2f * segmentTime;

            float localTime = gaitTime % extendedPeriod;

            float phaseA = 0f;
            float phaseB = 0f;

            if (localTime < halfT)
            {
                phaseA = 2f * Mathf.PI * localTime / T;
                phaseB = phaseA + Mathf.PI;
            }
            else if (localTime < segmentTime)
            {
                phaseA = Mathf.PI;
                phaseB = phaseA + Mathf.PI;
            }
            else if (localTime < segmentTime + halfT)
            {
                float t2 = localTime - segmentTime;
                phaseA = Mathf.PI + 2f * Mathf.PI * t2 / T;
                phaseB = phaseA + Mathf.PI;
            }
            else
            {
                phaseA = 2f * Mathf.PI;
                phaseB = phaseA + Mathf.PI;
            }

            SolveAndApply(phaseA, phaseB);

            // =========================================================
            // SPINE CONTROL
            // =========================================================

            if (spine == null || spine_Link == null)
                return;

            float A = spineAmplitudeDeg * Mathf.Deg2Rad;
            float spineAngleRad = 0f;

            if (activeSpineControl)
            {
                if (spineDriveDisabled)
                {
                    var drive = spine_Link.xDrive;
                    drive.stiffness = 1000f;
                    spine_Link.xDrive = drive;
                    spineDriveDisabled = false;
                }

                if (localTime < halfT)
                {
                    float ratio = localTime / halfT;
                    spineAngleRad = Mathf.Lerp(A, -A, ratio);
                }
                else if (localTime < segmentTime)
                {
                    spineAngleRad = -A;
                }
                else if (localTime < segmentTime + halfT)
                {
                    float t2 = localTime - segmentTime;
                    float ratio = t2 / halfT;
                    spineAngleRad = Mathf.Lerp(-A, A, ratio);
                }
                else
                {
                    spineAngleRad = A;
                }

                spine.SetAngle(spineAngleRad);
            }
            else
            {
                if (!spineDriveDisabled)
                {
                    var drive = spine_Link.xDrive;
                    drive.stiffness = 0f;
                    spine_Link.xDrive = drive;
                    spineDriveDisabled = true;
                }
            }
        }

        // =========================================================
        // IK SOLVER
        // =========================================================

        void SolveAndApply(float phaseA, float phaseB)
        {
            SolveAngles(
                phaseA, phaseB,
                out float yawA, out float hipA, out float kneeA,
                out float yawB, out float hipB, out float kneeB
            );

            ApplyAngles(yawA, hipA, kneeA, yawB, hipB, kneeB);
        }

        void SolveAngles(
            float phaseA, float phaseB,
            out float yawA, out float hipA, out float kneeA,
            out float yawB, out float hipB, out float kneeB)
        {
            Vector3 pRight = centerRight +
                new Vector3(Rx * Mathf.Cos(phaseA),
                            Ry * Mathf.Sin(phaseA), 0);

            Vector3 pLeft = centerLeft +
                new Vector3(Rx * Mathf.Cos(phaseB),
                            Ry * Mathf.Sin(phaseB), 0);

            InverseKinematics(pRight, a, b, c, d,
                              out yawA, out hipA, out kneeA);

            InverseKinematics(pLeft, a, b, c, d,
                              out yawB, out hipB, out kneeB);
        }

        void ApplyAngles(
            float yawA, float hipA, float kneeA,
            float yawB, float hipB, float kneeB)
        {
            hyFL.SetAngle(Mathf.PI + yawA);
            hyRR.SetAngle(-Mathf.PI - yawA);
            hyFR.SetAngle(-Mathf.PI - yawB);
            hyRL.SetAngle(Mathf.PI + yawB);

            hpFL.SetAngle(hipA);
            hpRR.SetAngle(hipA);
            hpFR.SetAngle(hipB);
            hpRL.SetAngle(hipB);

            kFL.SetAngle(kneeA);
            kRR.SetAngle(kneeA);
            kFR.SetAngle(kneeB);
            kRL.SetAngle(kneeB);
        }

        // =========================================================
        // IK MATH
        // =========================================================

        void InverseKinematics(
            Vector3 p,
            float a, float b, float c, float d,
            out float theta1,
            out float theta3,
            out float theta4)
        {
            float x = p.x;
            float y = p.y;
            float z = p.z;

            float r2 = x * x + z * z;
            float inside = Mathf.Max(0f, r2 - b * b);
            float Lmag = Mathf.Sqrt(inside);

            float t1a = Mathf.Atan2(z, -x) - Mathf.Atan2(b, Lmag);
            float t1b = Mathf.Atan2(z, -x) - Mathf.Atan2(b, -Lmag);

            t1a = Normalize(t1a);
            t1b = Normalize(t1b);

            float refAngle = -0.5f * Mathf.PI;
            theta1 = Mathf.Abs(AngleDiff(t1b, refAngle)) <
                     Mathf.Abs(AngleDiff(t1a, refAngle)) ? t1b : t1a;

            float L = -(x * Mathf.Cos(theta1) - z * Mathf.Sin(theta1));
            float Xp = L - a;
            float Yp = -y;

            float dist2 = Xp * Xp + Yp * Yp;
            float D = (dist2 - c * c - d * d) / (2f * c * d);
            D = Mathf.Clamp(D, -1f, 1f);

            float s = Mathf.Sqrt(Mathf.Max(0f, 1f - D * D));
            theta4 = Mathf.Atan2(s, D);

            theta3 = Mathf.Atan2(Yp, Xp)
                   - Mathf.Atan2(d * Mathf.Sin(theta4),
                                 c + d * Mathf.Cos(theta4));

            theta3 = Normalize(theta3);
            theta4 = Normalize(theta4);
        }

        float Normalize(float angle)
        {
            while (angle > Mathf.PI) angle -= 2f * Mathf.PI;
            while (angle < -Mathf.PI) angle += 2f * Mathf.PI;
            return angle;
        }

        float AngleDiff(float a, float b)
        {
            return Normalize(a - b);
        }

        public override void OnReset()
        {
            time = 0f;
        }
    }
}