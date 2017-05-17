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

        [Tooltip("Lean strength along each axis")]
        public Vector3 leanFactor;

        [Range(0, 0.99f)]
        public float leanSmoothness;

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
                    Vector3.Lerp(GlobalControl.worldUpDir, vp.wheelNormalAverage, Mathf.Abs(Vector3.Dot(vp.norm.forward, GlobalControl.worldUpDir)) * 2),
                    vp.wheelNormalAverage, normalStickiness));
                //inverseWorldUp = vp.wheelNormalAverage;
                //inverseWorldUp = vp.norm.InverseTransformDirection(vp.wheelNormalAverage);
                //Debug.DrawRay(tr.position, vp.wheelNormalAverage, Color.white);
                //Debug.DrawRay(tr.position, GlobalControl.worldUpDir, Color.green);
                //Debug.Log(Mathf.Abs(Vector3.Dot(vp.norm.up, GlobalControl.worldUpDir)) * 2);

                //Calculate target lean direction
                targetLean = new Vector3(
                    Mathf.Lerp(inverseWorldUp.x, 
                        Mathf.Clamp(-vp.rollInput * leanFactor.z * leanRollCurve.Evaluate(Mathf.Abs(vp.localVelocity.z)) + 
                                Mathf.Clamp(vp.localVelocity.x * slideLeanFactor, -leanFactor.z * slideLeanFactor, leanFactor.z * slideLeanFactor), 
                            -leanFactor.z, leanFactor.z), 
                        Mathf.Max(Mathf.Abs(F.MaxAbs(vp.steerInput, vp.rollInput)))),
                    0,
                    inverseWorldUp.z);
            }
            else
            {
                targetLean = vp.upDir;
            }

            //Transform targetLean to world space
            //targetLeanActual = Vector3.Lerp(targetLeanActual, vp.norm.TransformDirection(targetLean), (1 - leanSmoothness) * Time.timeScale * TimeMaster.inverseFixedTimeFactor).normalized;
            targetLeanActual = vp.norm.TransformDirection(targetLean);
            Debug.DrawRay(tr.position, targetLeanActual, Color.black);
            Debug.DrawRay(tr.position, tr.up, Color.red);

            /*//Apply pitch
            rb.AddTorque(
                vp.norm.right * -(Vector3.Dot(vp.forwardDir, targetLeanActual) * 20 - vp.localAngularVel.x) * 100 * (vp.wheels.Length == 1 ? 1 : leanPitchCurve.Evaluate(Mathf.Abs(actualPitchInput)))
                , ForceMode.Acceleration);

            //Apply yaw
            rb.AddTorque(
                vp.norm.forward * (vp.groundedWheels == 1 ? vp.steerInput * leanFactor.y - vp.norm.InverseTransformDirection(rb.angularVelocity).z : 0) * 100 * leanYawCurve.Evaluate(Mathf.Abs(vp.steerInput))
                , ForceMode.Acceleration);*/

            /*//Apply roll
            float rollAccel = (-Vector3.Dot(vp.rightDir, targetLeanActual) * 20 - vp.localAngularVel.z) * (1 / Time.fixedDeltaTime);//What does this magic number mean?
            //rollAccel = Mathf.Clamp(rollAccel, -maxLeanForce, maxLeanForce);
            rb.AddTorque(
                vp.norm.up * rollAccel
                , ForceMode.Acceleration);*/
            
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