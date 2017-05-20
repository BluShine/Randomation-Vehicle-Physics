using UnityEngine;
using System.Collections;

namespace RVP
{
    [RequireComponent(typeof(VehicleParent))]
    [DisallowMultipleComponent]
    [AddComponentMenu("RVP/Vehicle Controllers/Vehicle Balance", 4)]

    //Class for balancing vehicles
    public class VehicleBalance : MonoBehaviour
    {
        Transform tr;
        Rigidbody rb;
        VehicleParent vp;

        float actualPitchInput;
        Vector3 targetLean;
        Vector3 targetLeanActual;
        Vector3 lastLean;

        [Tooltip("Lean strength along each axis")]
        public Vector3 leanFactor;

        [Tooltip("Linear limit on fast you can change lean directions")]
        public float maxLeanSpeed = 1;

        [Tooltip("Adjusts the roll based on the speed, x-axis = speed, y-axis = roll amount")]
        public AnimationCurve leanRollCurve = AnimationCurve.Linear(0, 0, 10, 1);

        [Tooltip("How much to lean when sliding sideways")]
        public float slideLeanFactor = 1;

        public float maxBalanceAccel = 50;
        //PID balance controller
        public float gainP = 20;
        public float gainI = 1;
        public float gainD = 1;

        float balanceIntegrator = 0;
        float balanceLast = 0;

        [Range(0, 1f), Tooltip("How much the vehicle leans while driving along a cambered road")]
        public float normalStickiness = 0;

        void Start()
        {
            tr = transform;
            rb = GetComponent<Rigidbody>();
            vp = GetComponent<VehicleParent>();
            lastLean = Vector3.up;
        }

        void FixedUpdate()
        {
            if (vp.groundedWheels > 0)
            {
                if (leanFactor != Vector3.zero)
                {
                    ApplyLean();
                }
            }
        }

        void ApplyLean()
        {
            if (vp.groundedWheels > 0)
            {
                Vector3 inverseWorldUp;
                inverseWorldUp = vp.norm.InverseTransformDirection(Vector3.Lerp(
                    Vector3.Lerp(GlobalControl.worldUpDir, vp.wheelNormalAverage, Mathf.Abs(Vector3.Dot(tr.forward, GlobalControl.worldUpDir)) * 2),
                    vp.wheelNormalAverage, normalStickiness));

                //Calculate target lean direction
                targetLean = new Vector3(
                    Mathf.Lerp(inverseWorldUp.x,
                        inverseWorldUp.x + Mathf.Clamp(-vp.steerInput * leanFactor.z * leanRollCurve.Evaluate(Mathf.Abs(vp.localVelocity.z)),
                            -leanFactor.z, leanFactor.z),
                        Mathf.Max(Mathf.Abs(vp.steerInput))),
                    0,
                    inverseWorldUp.z);
            }
            else
            {
                targetLean = vp.upDir;
            }

            //Transform targetLean to world space
            targetLeanActual = vp.norm.TransformDirection(targetLean);
            float leanMag = Vector3.Magnitude(targetLeanActual - lastLean);
            if (leanMag > maxLeanSpeed * Time.fixedDeltaTime)
                targetLeanActual = Vector3.Lerp(lastLean, targetLeanActual, (maxLeanSpeed * Time.fixedDeltaTime) / leanMag);
            lastLean = targetLeanActual;

            Debug.DrawRay(tr.position, targetLeanActual, Color.black);
            Debug.DrawRay(tr.position, tr.up, Color.red);
            
            //figure out the z angle between our current rotation and target 
            float targetAngle = tr.InverseTransformDirection(Vector3.Cross(tr.up, targetLeanActual)).z;
            //calculate angular speed and accel for PID controller
            balanceIntegrator += targetAngle * Time.fixedDeltaTime;
            var diff = (targetAngle - balanceLast);
            balanceLast = targetAngle;
            //PID controller torque
            float torque = targetAngle * gainP + balanceIntegrator * gainI + diff * gainD;
            torque = Mathf.Clamp(torque, -maxBalanceAccel, maxBalanceAccel);
            rb.AddRelativeTorque(new Vector3(0, 0, torque), ForceMode.Acceleration);

            //Turn vehicle during wheelies
            if (vp.groundedWheels == 1 && leanFactor.y > 0)
            {
                rb.AddTorque(vp.norm.TransformDirection(new Vector3(
                    0,
                    0,
                    vp.steerInput * leanFactor.y - vp.norm.InverseTransformDirection(rb.angularVelocity).z
                    )), ForceMode.Acceleration);
            }
        }
    }
}